#include <memory>

#include "rclcpp/rclcpp.hpp"

class SampleNodeWithParameters : public rclcpp::Node
{ 
public:
    SampleNodeWithParameters()
        : Node("node_with_parameters")
    {
        this->declare_parameter("an_int_param", 0);

        // Create a parameter subscriber that can be used to monitor parameter changes
        // (for this node's parameters as well as other nodes' parameters)
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        auto cb = [this](const rcl_interfaces::msg::ParameterEvent &event)
        {
            auto params = rclcpp::ParameterEventHandler::get_parameters_from_event(event);
            for (auto &p : params)
            {
                std::cout << "here";
                RCLCPP_INFO(this->get_logger(), "cb3: Received an update to parameter \"%s\" of type: %s: %s",
                            p.get_name().c_str(), p.get_type_name().c_str(), p.value_to_string().c_str());
            }
        };

        cb_handle = param_subscriber_->add_parameter_event_callback(cb);
    }

private:
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> cb_handle;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SampleNodeWithParameters>());
    rclcpp::shutdown();

    return 0;
}
