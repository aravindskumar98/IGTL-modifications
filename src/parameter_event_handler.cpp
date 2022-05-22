#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// This node contains handles for monitoring changes to parameter values (created after this node's creation)

// TODO: // Todo : https://docs.ros.org/en/galactic/Tutorials/Intra-Process-Communication.html

class SampleNodeWithParameters : public rclcpp::Node
{
public:
    SampleNodeWithParameters()
        : Node("node_with_parameters"), count_(0)
    {
        const auto node_graph_interface = this->get_node_graph_interface();

        // Obtains the names of all active nodes in the environment
        std::vector<std::string> node_names = node_graph_interface->get_node_names();

        // Create a publisher to publish each time a change happens to a parameter value
        publisher_ = this->create_publisher<std_msgs::msg::String>("param_change_topic", 10);


        for (auto existing_node_name : node_names)
        {
            // Print names of all existing nodes
            RCLCPP_INFO(this->get_logger(), "Node Name : %s", existing_node_name.c_str());
        }

        // Create a parameter subscriber that can be used to monitor parameter changes
        // (for this node's parameters as well as other nodes' parameters)
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);


        auto cb = [this](const rcl_interfaces::msg::ParameterEvent &event)
        {
            // Obtain list of parameters that changed
            auto params = rclcpp::ParameterEventHandler::get_parameters_from_event(event);

            // Iterate through every parameter in the list
            for (auto &p : params)
            {
                RCLCPP_INFO(this->get_logger(), "cb3: Received an update to a parameter"); 

                // Create a string message from the info received.
                std::string msgString = "Param Name :" + p.get_name() + " | Param Type : " + p.get_type_name() + " | Param Value : " + p.value_to_string();
                
                // Publish the message
                this->timer_callback(msgString);
            }
        };

        cb_handle = param_subscriber_->add_parameter_event_callback(cb);
    }

private:

    // function to publish to the topic
    void timer_callback(std::string msgString)
    {
        auto message = std_msgs::msg::String();
        message.data = " || " +  msgString; 
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    // declarations
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
    return 0;
}
