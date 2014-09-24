#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <platform-abstraction/threading.h>
#include <platform-abstraction/panic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>

#include <stdarg.h>

#define NODE_ID 42
#define NODE_NAME "node-test-1"
#define CAN_BITRATE 1000000

uavcan_stm32::CanInitHelper<128> can;

typedef uavcan::Node<16384> Node;

uavcan::LazyConstructor<Node> node_;


Node& getNode()
{
    if (!node_.isConstructed()) {
        node_.construct<uavcan::ICanDriver&, uavcan::ISystemClock&>(can.driver, uavcan_stm32::SystemClock::instance());
    }
    return *node_;
}

void can2_gpio_init(void)
{
    // enable CAN transceiver
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
    gpio_clear(GPIOC, GPIO5);
    gpio_set_af(GPIOB, GPIO_AF9, GPIO12 | GPIO13);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12 | GPIO13);
}

void key_value_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue>& msg)
{
    const uint8_t *m = &msg.key[0];
    std::printf("KeyValue:%d: %s\n", msg.getSrcNodeID().get(), m);
}

void node_status_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg)
{
    const uint8_t st = msg.status_code;
    const char *st_name;
    switch (st) {
        case 0:
            st_name = "STATUS_OK";
            break;
        case 1:
            st_name = "STATUS_INITIALIZING";
            break;
        case 2:
            st_name = "STATUS_WARNING";
            break;
        case 3:
            st_name = "STATUS_CRITICAL";
            break;
        case 15:
            st_name = "STATUS_OFFLINE";
            break;
        default:
            st_name = "UNKNOWN_STATUS";
            break;
    }
    std::printf("NodeStatus from %d: %u (%s)\n", msg.getSrcNodeID().get(), st, st_name);
}

void log_cub_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage>& msg)
{
    const uint8_t *s = &msg.source[0];
    const uint8_t *m = &msg.text[0];
    const uint8_t l = msg.level.value;
    std::printf("LogMessage:%d: %s:%s, level: %d\n", msg.getSrcNodeID().get(), s, m, l);
}

void cpp_node_main(void)
{
    std::printf("Node Thread main\n");

    can2_gpio_init();
    std::printf("can lowlevel init\n");

    int init = can.init(CAN_BITRATE);

    if (init != 0) {
        std::printf("can driver init error\n");
        while(1);
    }
    std::printf("can driver init\n");

    /*
     * Setting up the node parameters
     */
    Node& node = getNode();

    node.setNodeID(NODE_ID);
    node.setName(NODE_NAME);

    std::printf("Node init %u\n", node.getNodeID().get());

    /*
     * Initializing the UAVCAN node - this may take a while
     */
    while (true) {
        int res = node.start();

        if (res < 0) {
            std::printf("Node initialization failure: %i, will try agin soon\n", res);
        } else {
            break;
        }
        os_thread_sleep_us(1000);
    }

    uavcan::Publisher<uavcan::protocol::debug::KeyValue> kv_pub(node);
    const int kv_pub_init_res = kv_pub.init();
    if (kv_pub_init_res < 0) {
        std::printf("error KeyValue publisher init");
        while (1);
    }


    uavcan::Subscriber<uavcan::protocol::debug::KeyValue> kv_sub(node);

    const int kv_sub_start_res = kv_sub.start(key_value_cb);

    if (kv_sub_start_res < 0)
    {
        std::printf("error KeyValue subscriber init");
        while (1);
    }

    /* node status subscriber */
    uavcan::Subscriber<uavcan::protocol::NodeStatus> ns_sub(node);

    const int ns_sub_start_res = ns_sub.start(node_status_cb);

    if (ns_sub_start_res < 0)
    {
        std::printf("error NodeStatus subscriber init");
        while (1);
    }

    /* log message subscriber */
    uavcan::Subscriber<uavcan::protocol::debug::LogMessage> log_sub(node);

    const int log_sub_start_res = log_sub.start(log_cub_cb);

    if (log_sub_start_res < 0)
    {
        std::printf("error LogMessage subscriber init");
        while (1);
    }

    /* logger */
    node.getLogger().setLevel(uavcan::protocol::debug::LogLevel::DEBUG);

    /*
     * Informing other nodes that we're ready to work.
     * Default status is INITIALIZING.
     */
    node.setStatusOk();

    /*
     * Main loop
     */
    std::printf("UAVCAN node started\n");

    while (true) {
        /*
         * Spinning for 1 second.
         * The method spin() may return earlier if an error occurs (e.g. driver failure).
         * All error codes are listed in the header uavcan/error.hpp.
         */
        int spin_res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));

        if (spin_res < 0) {
            std::printf("Spin failure: %i\n", spin_res);
        }

        uavcan::protocol::debug::KeyValue kv_msg;  // Always zero initialized
        kv_msg.type = kv_msg.TYPE_STRING;
        kv_msg.key = NODE_NAME;

        const int pub_res = kv_pub.broadcast(kv_msg);
        if (pub_res < 0)
        {
            std::printf("KV publication failure: %d\n", pub_res);
        }

        node.logInfo("main", "Hello world!");

        gpio_toggle(GPIOA, GPIO8);
    }
}

/*
int uavcan_stm32_log(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    printf("stm32_can: ");
    vprintf(fmt, ap);
    printf("\n");

    va_end(ap);
}
*/

/*
void uavcan_assert(const char *file, int line, bool cond)
{
    if (!cond) {
        std::printf("%s:%d: assert failed\n", file, line);
        while(1);
    }
}
*/

extern "C"
void node_main(void *arg)
{
    (void) arg;
    cpp_node_main();
}
