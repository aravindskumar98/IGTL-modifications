/*=========================================================================

  Program:   Bridge Converter Manager
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include "rib_converter_manager.h"
#include "rib_converter_base.h"

RIBConverterManager::RIBConverterManager()
{
  this->socket = NULL;
  //this->nh = NULL;
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
  //std::cerr << "void ConverterManager::AddConverter() topic = " << topicPublish << std::endl;
  converter->setup(this->node, size);
  converter->setManager(this);
  converter->publish(topicPublish);
  converter->subscribe(topicSubscribe);
  this->converters.push_back(converter);
}

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


void RIBConverterManager::sendROSMessage(cpp_parameter_event_handler::msg::String::SharedPtr msg)
{
  igtl::Socket::Pointer socket = this->GetSocket();
  std::cout<< "onROSMessage (String): "<< msg->name <<std::endl;     
  
  if (socket.IsNull())
    {
      std::cout<<"Socket Empty!"<<std::endl;
    return;
    }

  std::cout << "Socket found : "<<socket<<std::endl;
  //  SendString(msg->name.c_str(), msg->data);
  igtl::StringMessage::Pointer stringMsg = igtl::StringMessage::New();
  stringMsg->SetDeviceName(msg->name.c_str());
  stringMsg->SetString((msg->data).c_str());  
  stringMsg->Pack();
  //std::cout<<stringmsg.c_str()<< " sent"<<std::endl;
  socket->Send(stringMsg->GetPackPointer(), stringMsg->GetPackSize());
  std::cout << "Message Sent : "<<socket<<std::endl;
}
