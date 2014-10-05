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
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/protocol/global_time_sync_master.hpp>

#define MASTER 1

#if MASTER
#define NODE_ID 42
#define NODE_NAME "Master Node"
#else
#define NODE_ID 23
#define NODE_NAME "Slave Node"
#endif

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


    Node& node = getNode();

    node.setNodeID(NODE_ID);
    node.setName(NODE_NAME);

    uavcan::UtcDuration adjustment;
    uint64_t utc_time_init = 1234;
    adjustment = uavcan::UtcTime::fromUSec(utc_time_init) - uavcan::UtcTime::fromUSec(0);
    // adjustment.fromUSec(0);
    node.getSystemClock().adjustUtc(adjustment);

    std::printf("Node init %u\n", node.getNodeID().get());

    while (true) {
        int res = node.start();

        if (res < 0) {
            std::printf("Node initialization failure: %i, will try agin soon\n", res);
        } else {
            break;
        }
        os_thread_sleep_us(1000);
    }

#if MASTER
    /* time sync master */
    uavcan::GlobalTimeSyncMaster master(node);
    const int master_init_res = master.init();
    if (master_init_res < 0) {
        std::printf("error TimeSyncMaster init\n");
        while(1);
    }
#else
    /* time sync slave */
    uavcan::GlobalTimeSyncSlave slave(node);
    const int slave_init_res = slave.start();
    if (slave_init_res < 0) {
        std::printf("error TimeSyncSlave init\n");
        while(1);
    }
#endif

    /* node status subscriber */
    uavcan::Subscriber<uavcan::protocol::NodeStatus> ns_sub(node);
    const int ns_sub_start_res = ns_sub.start(node_status_cb);
    if (ns_sub_start_res < 0) {
        std::printf("error NodeStatus subscriber init");
        while (1);
    }

    std::printf("UAVCAN node started\n");

    node.setStatusOk();

    while (true) {
        int spin_res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));

        if (spin_res < 0) {
            std::printf("Spin failure: %i\n", spin_res);
        }

#if MASTER
        const int res = master.publish();
        if (res < 0) {
            std::printf("Time sync master transient failure: %d\n", res);
        }

        uavcan::MonotonicTime mono_time = node.getMonotonicTime();
        uavcan::UtcTime utc_time = node.getUtcTime();
        std::printf("Time: Monotonic: %llu   UTC: %llu\n",
                    mono_time.toUSec(), utc_time.toUSec());
#else
        const bool active = slave.isActive();
        const int master_node_id = slave.getMasterNodeID().get();
        const long msec_since_last_adjustment = (node.getMonotonicTime() - slave.getLastAdjustmentTime()).toMSec();
        std::printf("Time sync slave status:\n"
                    "    Master Node ID: %d, Active: %d\n"
                    "    Last adjust %ld ms ago, Monotonic: %llu   UTC: %llu\n\n",
                    master_node_id, int(active), msec_since_last_adjustment,
                    node.getMonotonicTime().toUSec(), node.getUtcTime().toUSec());

        // std::printf("Time: Monotonic: %llu, UTC: %llu\n", node.getMonotonicTime().toUSec(), node.getUtcTime().toUSec());
#endif

        gpio_toggle(GPIOA, GPIO8);
    }
}

/*
#include <stdarg.h>

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
