#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <csignal>
#include <aubo_hardware_interface.h>

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;
using RtdeRecipeMap =
    std::unordered_map<int, arcs::common_interface::RtdeRecipe>;

std::unique_ptr<aubo_driver::AuboHardwareInterface> g_aubo_hw_interface;

void signalHandler(int signum)
{
    
    std::cout << "Interrupt signal (" << signum << ") received.\n";

    // cleanup and close up stuff here
    // terminate program

    exit(signum);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aubo_hardware_interface");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle nh;
    ros::NodeHandle robot_hw_nh("~");

    signal(SIGINT, signalHandler);

    //    std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
    //    bool has_realtime = false;
    //    if (realtime_file.is_open()) {
    //        realtime_file >> has_realtime;
    //    }
    if (/*has_realtime*/ 1) {
        printf("[D] entering main loop\n");
        const int max_thread_priority = sched_get_priority_max(SCHED_FIFO);
        if (max_thread_priority != -1) {
            printf("[D] max thread priority is != -1\n");
            // We'll operate on the currently running thread.
            pthread_t this_thread = pthread_self();

            // struct sched_param is used to store the scheduling priority
            struct sched_param params;

            // We'll set the priority to the maximum.
            params.sched_priority = max_thread_priority;

            printf("[D] 1\n");
            int ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
            if (ret != 0) {
                printf("[D] 1a\n");
                ROS_ERROR_STREAM("Unsuccessful in setting main thread realtime "
                                 "priority. Error code: "
                                 << ret);
            }
            printf("[D] 2\n");
            // Now verify the change in thread priority
            int policy = 0;
            ret = pthread_getschedparam(this_thread, &policy, &params);
            if (ret != 0) {
                printf("[D] 2a\n");
                std::cout << "Couldn't retrieve real-time scheduling paramers"
                          << std::endl;
            }
            printf("[D] 3\n");

            // Check the correct policy was applied
            if (policy != SCHED_FIFO) {
                printf("[D] 3a\n");
                ROS_ERROR("Main thread: Scheduling is NOT SCHED_FIFO!");
            } else {
                printf("[D] 3b\n");
                ROS_INFO("Main thread: SCHED_FIFO OK");
            }

            // Print thread scheduling priority
            ROS_INFO_STREAM("Main thread priority is "
                            << params.sched_priority);
        } else {
            printf("[D] 4\n");
            ROS_ERROR("Could not get maximum thread priority for main thread");
        }
    }
    printf("[D] 5\n");
    // Set up timers
    ros::Time timestamp;
    ros::Duration period;
    auto stopwatch_last = std::chrono::steady_clock::now();
    auto stopwatch_now = stopwatch_last;

    g_aubo_hw_interface.reset(
        new aubo_driver::AuboHardwareInterface(robot_hw_nh));

    printf("[D] 6\n");
    if (!g_aubo_hw_interface->init(nh, robot_hw_nh)) {
        ROS_ERROR_STREAM("Could not correctly initialize robot. Exiting");
        exit(1);
    }
    printf("[D] 7\n");
    ROS_DEBUG_STREAM("initialized hw interface");
    controller_manager::ControllerManager cm(g_aubo_hw_interface.get(), nh);
    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(
                       stopwatch_now - stopwatch_last)
                       .count());
    stopwatch_last = stopwatch_now;
    printf("[D] 8\n");

    g_aubo_hw_interface->startServoMode();
    printf("[D] 9\n");
    
    ros::Rate _rate(250);  
    int loops=0;

    printf("[D] 10\n");
    while (ros::ok()) {

        printf("[D] loop\n");
        g_aubo_hw_interface->read(timestamp, period);
        timestamp = ros::Time::now();
        stopwatch_now = std::chrono::steady_clock::now();
        period.fromSec(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                stopwatch_now - stopwatch_last)
                .count());
        stopwatch_last = stopwatch_now;
        cm.update(timestamp, period,
                  g_aubo_hw_interface->shouldResetControllers());

        g_aubo_hw_interface->write(timestamp, period);

        if (g_aubo_hw_interface->writeret_ != 0)
        {
            ROS_FATAL("write failed with %d on loop %d",g_aubo_hw_interface->writeret_, ++loops);
            // break;
        }

        _rate.sleep();
    }
    printf("[D] 11\n");
    g_aubo_hw_interface->stopServoMode();
    spinner.stop();

    return 0;
}
