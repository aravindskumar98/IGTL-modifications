#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SampleNodeWithParameters : public rclcpp::Node
{
public:
    SampleNodeWithParameters()
        : Node("node_with_parameters"), count_(0)
    {
        const auto node_graph_interface = this->get_node_graph_interface();
        std::vector<std::string> node_names = node_graph_interface->get_node_names();

        publisher_ = this->create_publisher<std_msgs::msg::String>("param_change_topic", 10);


        for (auto existing_node_name : node_names)
        {
            // Is not working for some reason. Should look into it later
            //RCLCPP_INFO(this->get_logger(), "Node Name : %s", existing_node_name.c_str());
        }

        this->declare_parameter("an_int_param", 0);

        // Create a parameter subscriber that can be used to monitor parameter changes
        // (for this node's parameters as well as other nodes' parameters)
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        // RCLCPP_INFO(this->get_logger(), "new event!");
        auto cb = [this](const rcl_interfaces::msg::ParameterEvent &event)
        {
            auto params = rclcpp::ParameterEventHandler::get_parameters_from_event(event);
            for (auto &p : params)
            {
                RCLCPP_INFO(this->get_logger(), "cb3: Received an update to a parameter"); // \"%s\" of type: %s: %s",
                            //p.get_name().c_str(), p.get_type_name().c_str(), p.value_to_string().c_str());
                std::string msgString = "Param Name :" + p.get_name() + " | Param Type : " + p.get_type_name() + " | Param Value : " + p.value_to_string();
                
                this->timer_callback(msgString);
            }
        };

        cb_handle = param_subscriber_->add_parameter_event_callback(cb);
    }

private:
    void timer_callback(std::string msgString)
    {
        auto message = std_msgs::msg::String();
        // RCLCPP_INFO(this->get_logger()," String Concatenated :  %s", msgString.c_str());
        message.data = " || " +  msgString; //std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> cb_handle;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SampleNodeWithParameters>());
    rclcpp::shutdown();
// th
    return 0;
}
