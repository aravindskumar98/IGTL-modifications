
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "parameter_event_handler.h"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ParamBridge>();
    // node->addConverters();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
