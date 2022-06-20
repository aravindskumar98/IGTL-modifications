#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "igtlSocket.h"
#include "igtlClientSocket.h"
#include "igtlServerSocket.h"
#include "rib_converter_manager.h"

#define IGTL_DEFAULT_NODE_NAME "igtl_node"
using std::placeholders::_1;

using namespace std::chrono_literals;

class ParamBridge : public rclcpp::Node
{
public:
    ParamBridge();

protected:
    void timer_callback_msg(std::string msgString);

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> cb_handle;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_msg;
    // size_t count_;

protected:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

public:
    void addConverters();
    bool triggerSend();

    igtl::Socket::Pointer GetSocketPointer();
    std::string globalStr = "dummyStringValue"; // -------------- declaration
    int igtlActive = 0;

protected:
    virtual int StartIGTLServer();
    virtual int ConnectToIGTLServer();
    virtual void IGTLThread();

    igtl::Socket::Pointer socket;
    igtl::ServerSocket::Pointer serverSocket;
    igtl::ClientSocket::Pointer clientSocket;

    std::thread igtlThread;

    RIBConverterManager *converterManager;

    bool isServer; // Socket type for OpenIGTLink connection
    std::string address;
    int port;

private:
    // void topic_callback(std_msgs::msg::String::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    // void timer_callback();
};