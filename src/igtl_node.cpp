#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "igtl_node.h"

#include "rib_converter_manager.h"
#include "rib_converter_string.h"

//#include "rib_converter_polydata.h"

// #include "igtlOSUtil.h"

using namespace std::chrono_literals;


OpenIGTLinkNode::OpenIGTLinkNode() : Node(IGTL_DEFAULT_NODE_NAME), count_(0)
{
  //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  //timer_ = this->create_wall_timer(500ms, std::bind(&OpenIGTLinkNode::timer_callback, this));
}


OpenIGTLinkNode::OpenIGTLinkNode(const std::string nodeName) : Node(nodeName), count_(0)
{
  //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  //timer_ = this->create_wall_timer(500ms, std::bind(&OpenIGTLinkNode::timer_callback, this));
}


void OpenIGTLinkNode::addConverters()
{

  RCLCPP_ERROR(get_logger(), "Adding converters.");

  
  this->converterManager = new RIBConverterManager;
  rclcpp::Node::SharedPtr ptr = shared_from_this();
  //OpenIGTLinkNode::SharedPtr ptr = shared_from_this();
  this->converterManager->setNode(ptr);

  // Regisgter converter classes
  RIBConverterString* string = new RIBConverterString;

  // RIBConverterImage* image = new RIBConverterImage;
  // RIBConverterPointCloud* pointcloud = new RIBConverterPointCloud;  
  // RIBConverterPolyData* polydata = new RIBConverterPolyData;

  this->converterManager->AddConverter(string, 10, "IGTL_STRING_IN", "IGTL_STRING_OUT");
  // this->converterManager->AddConverter(image, 10, "IGTL_IMAGE_IN", "IGTL_IMAGE_OUT");
  // this->converterManager->AddConverter(pointcloud, 10, "IGTL_POINTCLOUD_IN", "IGTL_POINTCLOUD_OUT");
  //this->converterManager->AddConverter(polydata, 10, "IGTL_POLYDATA_IN", "IGTL_POLYDATA_OUT");


  RCLCPP_ERROR(get_logger(), "Checking parameters.");  
  // run bridge as client or server
  std::string type;

  this->port     = 18944;  // std port
  this->isServer = true;


  RCLCPP_INFO(get_logger(), "ROS-IGTL-Bridge is up and Running.");
  
  // start OpenIGTLink thread
  this->igtlThread = std::thread(&OpenIGTLinkNode::IGTLThread, this);
  this->igtlThread.detach();
}


//----------------------------------------------------------------------
igtl::Socket::Pointer OpenIGTLinkNode::GetSocketPointer()
{
  igtl::Socket::Pointer socket_ptr = static_cast<igtl::Socket::Pointer>(socket);
  return socket_ptr;
}


//----------------------------------------------------------------------
int OpenIGTLinkNode::StartIGTLServer()
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
  
  RCLCPP_INFO(get_logger(), "Server socket created. Please connect to port: %d",port);
  
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
int OpenIGTLinkNode::ConnectToIGTLServer()
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

void OpenIGTLinkNode::IGTLThread()
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
      rclcpp::shutdown();// TODO: Can the thread shutdown the process?
      }
      
    igtl::MessageHeader::Pointer headerMsg;
    headerMsg = igtl::MessageHeader::New();
    igtlUint64 rs = 0;
    int loop = 1;

    RCLCPP_INFO(get_logger(), "Connection established. Start the IGTL loop..");
    while(loop)
      {
      headerMsg->InitPack();
      // receive packet
      bool timeout = false;
      rs = this->socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize(), timeout);
          
      if (rs == 0)
        {
        this->socket->CloseSocket();
        this->converterManager->SetSocket(NULL);
        loop = 0; // Terminate the thread.
        }
          
      if (rs != headerMsg->GetPackSize())
        continue;
          
      this->converterManager->ProcessIGTLMessage(headerMsg);
      }
    }
}


void OpenIGTLinkNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}


