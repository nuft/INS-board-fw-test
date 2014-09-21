#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/can.hpp>
#include <uavcan_stm32/clock.hpp>
#include <platform-abstraction/threading.h>

/**
 * These functions are platform dependent, so they are not included in this example.
 * Refer to the relevant platform documentation to learn how to implement them.
 */
static uavcan_stm32::CanDriver can_driver;

uavcan::ICanDriver& getCanDriver()
{
    return can_driver;
}

static uavcan_stm32::SystemClock system_clock;

uavcan::ISystemClock& getSystemClock()
{
    return system_clock;
}


/**
 * Memory pool size largely depends on the number of CAN ifaces and on application behavior.
 * Please read the documentation for the class uavcan::Node to learn more.
 */

typedef uavcan::Node<16384> Node;

/**
 * Node object will be constructed at the time of the first access.
 * Note that most library objects are noncopyable (e.g. publishers, subscribers, servers, callers, timers, ...).
 * Attempt to copy a noncopyable object causes compilation failure.
 */
static Node& getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
}

void cpp_node_main(void)
{
    std::printf("node_main\n");

    const int self_node_id = 42;

    /*
     * Node initialization.
     * Node ID and name are required; otherwise, the node will refuse to start.
     * Version info is optional.
     */
    Node& node = getNode();

    node.setNodeID(self_node_id);

    node.setName("org.uavcan.tutorials");

    uavcan::protocol::SoftwareVersion sw_version;  // Standard type uavcan.protocol.SoftwareVersion
    sw_version.major = 1;
    node.setSoftwareVersion(sw_version);

    uavcan::protocol::HardwareVersion hw_version;  // Standard type uavcan.protocol.HardwareVersion
    hw_version.major = 1;
    node.setHardwareVersion(hw_version);

    /*
     * Start the node.
     * All returnable error codes are listed in the header file uavcan/error.hpp.
     */
    while (true)
    {
        const int res = node.start();
        if (res < 0)
        {
            std::printf("Node start failed: %d, will retry\n", res);
            os_thread_sleep_us(1000);
        }
        else { break; }
    }

    /*
     * Perform a network compatibility check.
     * This step is not mandatory and, in fact, rarely needed (read the specs).
     * All returnable error codes are listed in the header file uavcan/error.hpp.
     */
    uavcan::NetworkCompatibilityCheckResult network_compat_check_result;
    while (true)
    {
        const int res = node.checkNetworkCompatibility(network_compat_check_result);
        if (res < 0)
        {
            std::printf("Network compatibility check failed: %d, will retry\n", res);
            os_thread_sleep_us(1000);
        }
        else { break; }
    }

    if (!network_compat_check_result.isOk())
    {
        /*
         * Possible reasons:
         *  - There's another node with the same Node ID.
         *  - There's at least one incompatible data type in use on at least one remote node.
         */
        std::printf("Network conflict with node %d\n",
                    static_cast<int>(network_compat_check_result.conflicting_node.get()));
        return;
    }

    /*
     * Informing other nodes that we're ready to work.
     * Default status is INITIALIZING.
     */
    node.setStatusOk();

    /*
     * Some logging.
     * Log formatting is not available in C++03 mode.
     */
    node.getLogger().setLevel(uavcan::protocol::debug::LogLevel::DEBUG);
    // node.logInfo("main", "Hello world! My Node ID: %*",
    //              static_cast<int>(node.getNodeID().get()));

    std::puts("Hello world!");

    /*
     * Node loop.
     * The thread should not block outside Node::spin().
     */
    while (true)
    {
        /*
         * If there's nothing to do, the thread blocks inside the driver's
         * method select() until the timeout expires or an error occurs (e.g. driver failure).
         * All error codes are listed in the header uavcan/error.hpp.
         */
        const int res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (res < 0)
        {
            std::printf("Transient failure: %d\n", res);
        }

        /*
         * Random status transitions.
         * In real applications, the status code shall reflect node health; this feature is very important.
         */
        const float random = std::rand() / float(RAND_MAX);
        if (random < 0.5)
        {
            node.setStatusOk();
        }
        else if (random < 0.8)
        {
            node.setStatusWarning();
        }
        else
        {
            node.setStatusCritical();  // So bad.
        }
    }
}

extern "C" {
void node_main(void *arg) {
    (void) arg;
    cpp_node_main();
}

}
