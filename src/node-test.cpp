#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <platform-abstraction/threading.h>

#include <uavcan/protocol/debug/KeyValue.hpp> // uavcan.protocol.debug.KeyValue


uavcan_stm32::CanInitHelper<128> can;

typedef uavcan::Node<16384> Node;

uavcan::LazyConstructor<Node> node_;

Node& getNode()
{
    if (!node_.isConstructed())
    {
        node_.construct<uavcan::ICanDriver&, uavcan::ISystemClock&>(can.driver, uavcan_stm32::SystemClock::instance());
    }
    return *node_;
}

void cpp_node_main(void)
{
    /*
     * Setting up the node parameters
     */
    Node& node = getNode();

    node.setNodeID(64);
    node.setName("org.uavcan.stm32_test_stm32");

    uavcan::protocol::SoftwareVersion sw_version;  // Standard type uavcan.protocol.SoftwareVersion
    sw_version.major = 1;
    node.setSoftwareVersion(sw_version);

    uavcan::protocol::HardwareVersion hw_version;  // Standard type uavcan.protocol.HardwareVersion
    hw_version.major = 1;
    node.setHardwareVersion(hw_version);

    /*
     * Initializing the UAVCAN node - this may take a while
     */
    while (true)
    {
        // Calling start() multiple times is OK - only the first successfull call will be effective
        int res = node.start();

#if !UAVCAN_TINY
        uavcan::NetworkCompatibilityCheckResult ncc_result;
        if (res >= 0)
        {
            std::printf("Checking network compatibility...\n");
            res = node.checkNetworkCompatibility(ncc_result);
        }
#endif

        if (res < 0)
        {
            std::printf("Node initialization failure: %i, will try agin soon\n", res);
        }
#if !UAVCAN_TINY
        else if (!ncc_result.isOk())
        {
            std::printf("Network conflict with %u, will try again soon\n", ncc_result.conflicting_node.get());
        }
#endif
        else
        {
            break;
        }
        os_thread_sleep_us(1000);
    }

    uavcan::Publisher<uavcan::protocol::debug::KeyValue> kv_pub(node);
    const int kv_pub_init_res = kv_pub.init();
    if (kv_pub_init_res < 0)
    {
        std::printf("error KeyValue publisher init");
        while (1);
    }

    kv_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));


    // uavcan::Subscriber<uavcan::protocol::debug::KeyValue> kv_sub(node);

    // const int kv_sub_start_res =
    //     kv_sub.start([&](const uavcan::protocol::debug::KeyValue& msg) { std::cout << msg << std::endl; });

    // if (kv_sub_start_res < 0)
    // {
    //     std::printf("error KeyValue subscriber init");
    //     while (1);
    // }

    /*
     * Main loop
     */
    std::printf("UAVCAN node started\n");
    node.setStatusOk();
    while (true)
    {
        /*
         * Spinning for 1 second.
         * The method spin() may return earlier if an error occurs (e.g. driver failure).
         * All error codes are listed in the header uavcan/error.hpp.
         */
        const int spin_res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (spin_res < 0)
        {
            std::printf("Spin failure: %i\n", spin_res);
        }

        /*
         * Publishing a random value using the publisher created above.
         * All message types have zero-initializing default constructors.
         * Relevant usage info for every data type is provided in its DSDL definition.
         */
        uavcan::protocol::debug::KeyValue kv_msg;  // Always zero initialized
        kv_msg.type = kv_msg.TYPE_FLOAT;
        kv_msg.numeric_value.push_back(3.141);

        /*
         * Arrays in DSDL types are quite extensive in the sense that they can be static,
         * or dynamic (no heap needed - all memory is pre-allocated), or they can emulate std::string.
         * The last one is called string-like arrays.
         * ASCII strings can be directly assigned or appended to string-like arrays.
         * For more info, please read the documentation for the class uavcan::Array<>.
         */
        kv_msg.key = "random";  // "random"
        kv_msg.key += "_";      // "random_"
        kv_msg.key += "float";  // "random_float"

        /*
         * Publishing the message. Two methods are available:
         *  - broadcast(message)
         *  - unicast(message, destination_node_id)
         * Here we use broadcasting.
         */
        const int pub_res = kv_pub.broadcast(kv_msg);
        if (pub_res < 0)
        {
            std::printf("KV publication failure: %d\n", pub_res);
        }
    }
}

extern "C" {
void node_main(void *arg) {
    (void) arg;
    cpp_node_main();
}

}
