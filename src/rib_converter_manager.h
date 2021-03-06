/*=========================================================================

  Program:   Converter Manager
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more converter_managerrmation.

=========================================================================*/

// This class is used to share the information 

#ifndef __rib_converter_manager_h
#define __rib_converter_manager_h

//#include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
#include "igtlSocket.h"
#include "igtlMessageHeader.h"
#include "cpp_parameter_event_handler/msg/string.hpp"
#include "igtlStringMessage.h"

class RIBConverterBase;

class RIBConverterManager
{
 public:
  RIBConverterManager();

  void SetSocket(igtl::Socket* socket) { this->socket = socket; };
  igtl::Socket::Pointer GetSocket();
  
  void setNode(rclcpp::Node::SharedPtr n) { this->node = n; }  
  
  void AddConverter(RIBConverterBase* converter, uint32_t size, const char* topicPublish, const char* topicSubscribe);

  
  void ProcessIGTLMessage(igtl::MessageHeader* headerMsg);
  void sendROSMessage(cpp_parameter_event_handler::msg::String::SharedPtr msg);
  
 protected:
  ~RIBConverterManager();
  
  igtl::Socket::Pointer socket;
  rclcpp::Node::SharedPtr node;
  std::vector< RIBConverterBase* > converters;
};

#endif // __rib_converter_manager_h
