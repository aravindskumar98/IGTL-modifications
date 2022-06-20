#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread> 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "parameter_event_handler.h"
#include "rib_converter_manager.h"

using namespace std::chrono_literals;

using namespace std::chrono_literals;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono;
// This node contains handles for monitoring changes to parameter values (created after this node's creation)

// TODO: // Todo : https://docs.ros.org/en/galactic/Tutorials/Intra-Process-Communication.html

ParamBridge::ParamBridge() : Node("parameter_bridge_node"), count_(0)
{
    const auto node_graph_interface = this->get_node_graph_interface();

    // Obtains the names of all active nodes in the environment
    std::vector<std::string> node_names = node_graph_interface->get_node_names();

    // Create a publisher to publish each time a change happens to a parameter value
    publisher_msg = this->create_publisher<std_msgs::msg::String>("param_change_topic", 10);

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
            this->timer_callback_msg(msgString);
        }
    };

    cb_handle = param_subscriber_->add_parameter_event_callback(cb);
}

// function to publish to the topic
void ParamBridge::timer_callback_msg(std::string msgString)
{
    auto message = std_msgs::msg::String();
    message.data = " || " + msgString;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_msg->publish(message);
}


void ParamBridge::addConverters()
{

  RCLCPP_ERROR(get_logger(), "Adding converters.");

  this->converterManager = new RIBConverterManager;
  rclcpp::Node::SharedPtr ptr = shared_from_this();
  this->converterManager->setNode(ptr);

  RCLCPP_ERROR(get_logger(), "Checking parameters.");
  std::string type;

  this->port = 18944; // std port
  this->isServer = true; // Run as server

  RCLCPP_INFO(get_logger(), "ROS-IGTL-Bridge is up and Running.");

  // start OpenIGTLink thread
  this->igtlThread = std::thread(&ParamBridge::IGTLThread, this);
  this->igtlThread.detach();
}

//----------------------------------------------------------------------
igtl::Socket::Pointer ParamBridge::GetSocketPointer()
{
  igtl::Socket::Pointer socket_ptr = static_cast<igtl::Socket::Pointer>(socket);
  return socket_ptr;
}

//----------------------------------------------------------------------
int ParamBridge::StartIGTLServer()
{

  static igtl::ServerSocket::Pointer serverSocket;

  if (serverSocket.IsNull()) // if called for the first time
  {
    serverSocket = igtl::ServerSocket::New();
    int c = serverSocket->CreateServer(this->port);
    if (c < 0)
    {
      RCLCPP_ERROR(get_logger(), "Cannot create a server socket.");
      return 0;
    }
  }

  RCLCPP_INFO(get_logger(), "Server socket created. Please connect to port: %d", port);

  // wait for connection
  while (1)
  {
    RCLCPP_ERROR(get_logger(), "Waiting for connection.");
    this->socket = serverSocket->WaitForConnection(1000);
    this->converterManager->SetSocket(this->socket);
    if (this->socket.IsNotNull())
    {
      RCLCPP_ERROR(get_logger(), "Connected.");
      return 1;
    }
  }

  return 0;
}

//----------------------------------
int ParamBridge::ConnectToIGTLServer()
{
  static igtl::ClientSocket::Pointer clientSocket;

  if (clientSocket.IsNull()) // if called for the first time
  {
    clientSocket = igtl::ClientSocket::New();
  }

  // connect to server
  int r = clientSocket->ConnectToServer(this->address.c_str(), this->port);

  if (r != 0)
  {
    RCLCPP_ERROR(get_logger(), "Cannot connect to server.");
    return 0;
  }

  this->socket = (igtl::Socket *)(clientSocket);
  this->converterManager->SetSocket(this->socket);
  return 1;
}

// Prints the message received onto console. Waits and then initiates the sending process to IGTL
void ParamBridge::topic_callback(std_msgs::msg::String::SharedPtr msg)
{
  this->globalStr = msg->data;
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", globalStr.c_str()); 
  sleep_for(seconds(1));
  
  auto handle = std::async(std::launch::async, &ParamBridge::triggerSend, this); // Trying out an asynchronous call 

//   auto res = handle.get();
}

// Sends the message received to IGTL
bool ParamBridge::triggerSend()
{
  if (this->igtlActive == 0)
  {
    RCLCPP_INFO(this->get_logger(), "IGTL Node inactive. Retry connection");
    return 0;
  }
  cpp_parameter_event_handler::msg::String::SharedPtr msg_to_send;
  msg_to_send.reset(new (cpp_parameter_event_handler::msg::String));
  msg_to_send->name = "ParamEventHandler";
  msg_to_send->data = this->globalStr;

  this->converterManager->sendROSMessage(msg_to_send);
  return 0;
}

// Function to establish connection to IGTL via a socket
void ParamBridge::IGTLThread()
{
  int r;

  while (1)
  {
    if (this->isServer)
    {
      r = this->StartIGTLServer();
    }
    else
    {
      r = this->ConnectToIGTLServer();
    }
    if (r == 0)
    {
      RCLCPP_ERROR(get_logger(), "Failed to create a socket. Terminating...");
      rclcpp::shutdown(); // TODO: Can the thread shutdown the process?
    }

    igtl::MessageHeader::Pointer headerMsg;
    headerMsg = igtl::MessageHeader::New();
    // igtlUint64 rs = 0;
    int loop = 1;

    RCLCPP_INFO(get_logger(), "Connection established. Subscriber created. Start the IGTL loop.. ");
    this->igtlActive = 1;

    // Creating a subscriber to the param_change_topic.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "param_change_topic", 10, std::bind(&ParamBridge::topic_callback, this, _1));

    // Reach the loop
    RCLCPP_INFO(get_logger(), "Inside loop");
    while (loop)
    {

      // Using this as a way to keep the loop active for param event handler
      std::cout << " Waiting... " << std::endl;
      std::cin.get(); 

    }
  }
}

void ParamBridge::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}
