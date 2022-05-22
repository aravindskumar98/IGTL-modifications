/*=========================================================================

  Program:   Bridge Converter Manager
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

// This was used in to the ROS2-IGTL-Bridge repo for sending and receiving messages of different types using converters

// For the parameter changes, a new function sendROSMessage has been created. That sends an input message to the IGTL side.


#include "rib_converter_manager.h"
#include "rib_converter_base.h"

RIBConverterManager::RIBConverterManager()
{
  this->socket = NULL;
  this->node = NULL;
  this->converters.clear();
}

RIBConverterManager::~RIBConverterManager()
{
}

igtl::Socket::Pointer RIBConverterManager::GetSocket()
{
  igtl::Socket::Pointer socket_ptr = static_cast<igtl::Socket::Pointer>(this->socket);
  return socket_ptr;
}

void RIBConverterManager::AddConverter(RIBConverterBase* converter, uint32_t size, const char* topicPublish, const char* topicSubscribe)
{
  converter->setup(this->node, size);
  converter->setManager(this);
  converter->publish(topicPublish);
  converter->subscribe(topicSubscribe);
  this->converters.push_back(converter);
}


// This function receives a message, packs it and sends it to IGTL
void RIBConverterManager::sendROSMessage(cpp_parameter_event_handler::msg::String::SharedPtr msg)
{ 
  // obtain socket
  igtl::Socket::Pointer socket = this->GetSocket();

  // display message to be send on console
  std::cout<< "onROSMessage (String): "<< msg->name <<std::endl;     
  
  if (socket.IsNull())
    {
      std::cout<<"Socket Empty!"<<std::endl;
    return;
    }

  // if Socket found. proceed ... 

  // create a new string message (This is composed of all the parameter data)
  igtl::StringMessage::Pointer stringMsg = igtl::StringMessage::New();

  stringMsg->SetDeviceName(msg->name.c_str());
  stringMsg->SetString((msg->data).c_str());  
  // pack the message for sending
  stringMsg->Pack();

  // message sent
  socket->Send(stringMsg->GetPackPointer(), stringMsg->GetPackSize());
  std::cout << "Message Sent : "<<socket<<std::endl;
}


// Function to process messages received at the socket
void RIBConverterManager::ProcessIGTLMessage(igtl::MessageHeader* headerMsg)
{
  headerMsg->Unpack();

  if (this->socket.IsNull())
    {
    return;
    }
  
  std::vector< RIBConverterBase* >::iterator iter;
  for (iter = this->converters.begin(); iter != this->converters.end(); iter ++)
    {
    if (strcmp(headerMsg->GetDeviceType(), (*iter)->messageTypeString()) == 0)
      {
      (*iter)->onIGTLMessage(headerMsg);
      break;
      }
    }
  if (iter == this->converters.end())
    {
    this->socket->Skip(headerMsg->GetBodySizeToRead(),0);
    }
}
