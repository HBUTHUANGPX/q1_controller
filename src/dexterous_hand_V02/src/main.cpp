#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "dexterous_hand/hand_node.hpp"

bool isQuit = false;
void signal_handler(int signal)
{
    // ARM_LOG_INFO("~exit~");
    std::cout << "[signal_handler] exit." << std::endl;
    isQuit = true;

    // void *array[50];
    // size_t size = backtrace(array, 50);

    // fprintf(stderr, "Error: signal %d:\n", signal);
    // backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

int main(int argc, char *argv[])
{

    signal(SIGINT, signal_handler);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandDriverNode>());
    rclcpp::shutdown();
    return 0;
}