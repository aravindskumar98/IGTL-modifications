/*=========================================================================

  Program:   Converter Base Class
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

// This Class adds a templated function (onROSMessagre) to
// the RIBConverterBase class.

#ifndef __RIBConverter_H
#define __RIBConverter_H

#include "rclcpp/rclcpp.hpp"

#include "rib_converter_base.h"
//#include "ros/callback_queue.h"
#include "igtlMessageHeader.h"
//#include "igtlSocket.h"

template <typename MessageType>
class RIBConverter : public RIBConverterBase
{

public:
  RIBConverter();
  RIBConverter(rclcpp::Node::SharedPtr n);
  RIBConverter(const char* topicPublish, const char* topicSubscribe, rclcpp::Node::SharedPtr n=NULL);
  
protected:
  virtual void onROSMessage(const typename MessageType::SharedPtr msg) = 0;

public:

  virtual bool publish(const char* topic);
  virtual bool subscribe(const char* topic);
  
protected:
  
  typename rclcpp::Publisher<MessageType>::SharedPtr publisher;
  typename rclcpp::Subscription<MessageType>::SharedPtr subscription;

  ~RIBConverter();

};

  
#include "rib_converter.tpp"


#endif // __RIBConverter_H
