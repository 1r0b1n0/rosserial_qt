/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_NODE_HANDLE_H_
#define ROS_NODE_HANDLE_H_

#include <stdint.h>

#include "std_msgs/Time.h"
#include "rosserial_msgs/TopicInfo.h"
#include "rosserial_msgs/Log.h"
#include "rosserial_msgs/RequestParam.h"
#include "ros/msg.h"

#include <QObject>

namespace ros {

class NodeHandleBase_ : public QObject{
  Q_OBJECT
public:
  NodeHandleBase_(QObject *parent=0) : QObject(parent){}

  virtual int publish(int id, const Msg* msg)=0;
  virtual int spinOnce()=0;
  virtual bool connected()=0;
};
}

#include "RosQtSocket.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/service_server.h"
#include "ros/service_client.h"

namespace ros {

const uint8_t SYNC_SECONDS  = 5;
const uint8_t MODE_FIRST_FF = 0;
/*
   * The second sync byte is a protocol version. It's value is 0xff for the first
   * version of the rosserial protocol (used up to hydro), 0xfe for the second version
   * (introduced in hydro), 0xfd for the next, and so on. Its purpose is to enable
   * detection of mismatched protocol versions (e.g. hydro rosserial_python with groovy
   * rosserial_arduino. It must be changed in both this file and in
   * rosserial_python/src/rosserial_python/SerialClient.py
   */
const uint8_t MODE_PROTOCOL_VER   = 1;
const uint8_t PROTOCOL_VER1       = 0xff; // through groovy
const uint8_t PROTOCOL_VER2       = 0xfe; // in hydro
const uint8_t PROTOCOL_VER        = PROTOCOL_VER2;
const uint8_t MODE_SIZE_B1         = 2; // LSB
const uint8_t MODE_SIZE_B2         = 3;
const uint8_t MODE_SIZE_B3         = 4;
const uint8_t MODE_SIZE_B4         = 5; // MSB
const uint8_t MODE_SIZE_CHECKSUM  = 6;    // checksum for msg size received from size L and H
const uint8_t MODE_TOPIC_L        = 7;    // waiting for topic id
const uint8_t MODE_TOPIC_H        = 8;
const uint8_t MODE_MESSAGE        = 9;
const uint8_t MODE_MSG_CHECKSUM   = 10;    // checksum for msg and topic id

const uint32_t SERIAL_MSG_TIMEOUT  = 2000;   // 2000 milliseconds to recieve all of message data
const uint32_t READ_BUFFER_SIZE = 4096; // at each read() we will ask up to READ_BUFFER_SIZE bytes

const size_t MESSAGE_HEADER_BYTES = 10; // size of the header
const size_t BUFFER_OVER_ALLOCATION = 100*1024; //if the tx or rx buffer hasn't enough capacity, we will allocate this more

using rosserial_msgs::TopicInfo;

/* Node Handle */
class NodeHandle : public NodeHandleBase_
{
  Q_OBJECT
protected:
  RosQtSocket hardware_;

  /* time used for syncing */
  uint32_t rt_time;

  /* used for computing current time */
  uint32_t sec_offset, nsec_offset;

  std::vector<uint8_t> message_in;
  std::vector<uint8_t> message_out;

  std::vector<Publisher_ *> publishers;
  std::vector<Subscriber_ *> subscribers;

  uint32_t INPUT_SIZE;
  uint32_t OUTPUT_SIZE;

  /*
       * Setup Functions
       */
public:
  NodeHandle(uint32_t input_size=10000, uint32_t output_size=10000, QObject *parent=0) :
    NodeHandleBase_(parent),
    INPUT_SIZE(input_size),
    OUTPUT_SIZE(output_size),
    configured_(false)
  {
    message_in.resize(INPUT_SIZE);
    message_out.resize(OUTPUT_SIZE);

    for(unsigned int i=0; i< INPUT_SIZE; i++)
      message_in[i] = 0;

    for(unsigned int i=0; i< OUTPUT_SIZE; i++)
      message_out[i] = 0;

    connect(&hardware_, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
    startTimer(100);
  }

  void timerEvent(QTimerEvent *event)
  {
    (void)event;
    spinOnce();
  }

  /* Start serial, initialize buffers */
  void initNode(){

  }

  /* Start a named port, which may be network server IP, initialize buffers */
  void initNode(const std::string &portName){
    hardware_.init(portName);
    mode_ = 0;
    bytes_ = 0;
    index_ = 0;
    topic_ = 0;
  }

protected:
  //State machine variables for spinOnce
  int mode_;
  uint32_t bytes_;
  int topic_;
  int index_;
  unsigned int checksum_;

  bool configured_;

  /* used for syncing the time */
  uint32_t last_sync_time;
  uint32_t last_sync_receive_time;
  uint32_t last_msg_timeout_time;

public:
  /* This function goes in your loop() function, it handles
       *  serial input and callbacks for subscribers.
       */


  virtual int spinOnce(){
    unsigned char buffer[READ_BUFFER_SIZE];

    /* restart if timed out */
    uint32_t c_time = hardware_.time();
    if( (c_time - last_sync_receive_time) > (SYNC_SECONDS*2200) ){
      configured_ = false;
    }

    /* reset if message has timed out */
    if ( mode_ != MODE_FIRST_FF){
      if (c_time > last_msg_timeout_time){
        mode_ = MODE_FIRST_FF;
      }
    }

    /* while available buffer, read data */
    while(true)
    {
      int64_t result = hardware_.read(buffer, READ_BUFFER_SIZE);
      if(result == 0)
      {
        // no more data
        break;
      }
      else if(result < 0)
      {
        // connection lost
        configured_ = false;
        return -2;
      }

      for(size_t i=0 ; i<(size_t)result ; i++)
      {
        unsigned char data = buffer[i];

        checksum_ += data;
        if( mode_ == MODE_MESSAGE ){        /* message data being recieved */
          message_in[index_++] = data;
          bytes_--;
          if(bytes_ == 0)                  /* is message complete? if so, checksum */
            mode_ = MODE_MSG_CHECKSUM;
        }else if( mode_ == MODE_FIRST_FF ){
          if(data == 0xff){
            mode_++;
            last_msg_timeout_time = c_time + SERIAL_MSG_TIMEOUT;
          }
          else if( hardware_.time() - c_time > (SYNC_SECONDS*1000)){
            /* We have been stuck in spinOnce too long, return error */
            configured_=false;
            return -2;
          }
        }else if( mode_ == MODE_PROTOCOL_VER ){
          if(data == PROTOCOL_VER){
            mode_++;
          }else{
            mode_ = MODE_FIRST_FF;
            if (configured_ == false)
              requestSyncTime(); 	/* send a msg back showing our protocol version */
          }
        }else if( mode_ == MODE_SIZE_B1 ){   /* bottom half of message size */
          bytes_ = data;
          index_ = 0;
          mode_++;
          checksum_ = data;               /* first byte for calculating size checksum */
        }else if( mode_ == MODE_SIZE_B2 ){   /* part 2 of message size */
          bytes_ += data<<8;
          mode_++;
        }else if( mode_ == MODE_SIZE_B3 ){   /* part 3 of message size */
          bytes_ += data<<16;
          mode_++;
        }else if( mode_ == MODE_SIZE_B4 ){   /* part 4 of message size */
          bytes_ += data<<24;
          mode_++;
        }
        else if( mode_ == MODE_SIZE_CHECKSUM ){
          if( (checksum_%256) == 255)
          {
            mode_++;
          }
          else
          {
            mode_ = MODE_FIRST_FF;          /* Abandon the frame if the msg len is wrong */
          }
        }else if( mode_ == MODE_TOPIC_L ){  /* bottom half of topic id */
          topic_ = data;
          mode_++;
          checksum_ = data;               /* first byte included in checksum */
        }else if( mode_ == MODE_TOPIC_H ){  /* top half of topic id */
          topic_ += data<<8;
          mode_ = MODE_MESSAGE;

          // if needed expand buffer
          if(bytes_ + MESSAGE_HEADER_BYTES > message_in.size())
          {
            message_in.resize(bytes_ + MESSAGE_HEADER_BYTES + BUFFER_OVER_ALLOCATION);
          }

          if(bytes_ == 0)
            mode_ = MODE_MSG_CHECKSUM;
        }else if( mode_ == MODE_MSG_CHECKSUM ){ /* do checksum */
          mode_ = MODE_FIRST_FF;
          if( (checksum_%256) == 255){
            if(topic_ == TopicInfo::ID_PUBLISHER){
              requestSyncTime();
              negotiateTopics();
              last_sync_time = c_time;
              last_sync_receive_time = c_time;
              return -1;
            }else if(topic_ == TopicInfo::ID_TIME){
              syncTime(message_in.data());
            }else if (topic_ == TopicInfo::ID_PARAMETER_REQUEST){
              req_param_resp.deserialize(message_in.data());
              param_recieved= true;
            }else if(topic_ == TopicInfo::ID_TX_STOP){
              configured_ = false;
            }else{
              if(subscribers[topic_-100])
                subscribers[topic_-100]->callback( message_in.data());
            }
          }
        }
      }
    }

    /* occasionally sync time */
    if( configured_ && ((c_time-last_sync_time) > (SYNC_SECONDS*500) )){
      requestSyncTime();
      last_sync_time = c_time;
    }

    return 0;
  }


  /* Are we connected to the PC? */
  virtual bool connected() {
    return configured_;
  }

  /********************************************************************
       * Time functions
       */

  void requestSyncTime()
  {
    std_msgs::Time t;
    publish(TopicInfo::ID_TIME, &t);
    rt_time = hardware_.time();
  }

  void syncTime(uint8_t * data)
  {
    std_msgs::Time t;
    uint32_t offset = hardware_.time() - rt_time;

    t.deserialize(data);
    t.data.sec += offset/1000;
    t.data.nsec += (offset%1000)*1000000UL;

    this->setNow(t.data);
    last_sync_receive_time = hardware_.time();
  }

  Time now()
  {
    uint32_t ms = hardware_.time();
    Time current_time;
    current_time.sec = ms/1000 + sec_offset;
    current_time.nsec = (ms%1000)*1000000UL + nsec_offset;
    normalizeSecNSec(current_time.sec, current_time.nsec);
    return current_time;
  }

  void setNow( Time & new_now )
  {
    uint32_t ms = hardware_.time();
    sec_offset = new_now.sec - ms/1000 - 1;
    nsec_offset = new_now.nsec - (ms%1000)*1000000UL + 1000000000UL;
    normalizeSecNSec(sec_offset, nsec_offset);
  }

  /********************************************************************
       * Topic Management
       */

  /* Register a new publisher */
  template<typename T>
  bool advertise(Publisher<T> & p)
  {
    size_t i = publishers.size();
    publishers.push_back(&p);
    p.id_ = i+100;
    p.nh_ = this;
    return true;
  }

  /* Register a new subscriber */
  template<typename SubscriberT>
  bool subscribe(SubscriberT& s){
    size_t i = subscribers.size();
    subscribers.push_back(static_cast<Subscriber_*>(&s));
    s.id_ = i+100;
    return true;
  }

  /* Register a new Service Server */
  template<typename MType>
  bool advertiseService(ServiceServer<MType>& srv){
    bool v = advertise(srv.pub);
    size_t i = subscribers.size();
    subscribers.push_back(static_cast<Subscriber_*>(&srv));
    srv.id_ = i+100;
    return v;
  }

  /* Register a new Service Client */
  template<typename MType>
  bool serviceClient(ServiceClient<MType>& srv){
    bool v = advertise(srv.pub);
    size_t i = subscribers.size();
    subscribers.push_back(static_cast<Subscriber_*>(&srv));
    srv.id_ = i+100;
    return v;
  }

  void negotiateTopics()
  {
    rosserial_msgs::TopicInfo ti;
    size_t i;
    for(i = 0; i < publishers.size(); i++)
    {
      if(publishers[i] != 0) // non-empty slot
      {
        ti.topic_id = publishers[i]->id_;
        ti.topic_name = publishers[i]->topic_;
        ti.message_type = publishers[i]->getMsgType();
        ti.md5sum = publishers[i]->getMsgMD5();
        ti.buffer_size = OUTPUT_SIZE;
        publish( publishers[i]->getEndpointType(), &ti );
      }
    }
    for(i = 0; i < subscribers.size(); i++)
    {
      if(subscribers[i] != 0) // non-empty slot
      {
        ti.topic_id = subscribers[i]->id_;
        ti.topic_name = subscribers[i]->topic_;
        ti.message_type = subscribers[i]->getMsgType();
        ti.md5sum = subscribers[i]->getMsgMD5();
        ti.buffer_size = INPUT_SIZE;
        publish( subscribers[i]->getEndpointType(), &ti );
      }
    }
    configured_ = true;
  }

  virtual int publish(int id, const Msg * msg)
  {
    if(id >= 100 && !configured_)
      return 0;

    /* serialize message */
    size_t expected_l = msg->serialize_size();

    if(expected_l + MESSAGE_HEADER_BYTES > message_out.size())
    {
      message_out.resize(expected_l + MESSAGE_HEADER_BYTES + BUFFER_OVER_ALLOCATION);
    }

    size_t l = msg->serialize(message_out.data()+9);

    if(expected_l != l)
    {
      logerror("Internal error : expected_l != l ; memory could be corrupted");
    }

    /* setup the header */
    message_out[0] = 0xff;
    message_out[1] = PROTOCOL_VER;
    message_out[2] = (uint8_t) ((uint32_t)l&255);
    message_out[3] = (uint8_t) ((uint32_t)l>>8);
    message_out[4] = (uint8_t) ((uint32_t)l>>16);
    message_out[5] = (uint8_t) ((uint32_t)l>>24);
    message_out[6] = 255 - ((message_out[2] + message_out[3] + message_out[4] + message_out[5])%256);
    message_out[7] = (uint8_t) ((int16_t)id&255);
    message_out[8] = (uint8_t) ((int16_t)id>>8);

    /* calculate checksum */
    int chk = 0;
    l += 9;
    for(size_t i =7; i<l; i++)
      chk += message_out[i];
    message_out[l++] = 255 - (chk%256);

    hardware_.write(message_out.data(), l);
    return l;
  }

private slots:
  void onReadyRead()
  {
    spinOnce();
  }

  /********************************************************************
       * Logging
       */

private:
  void log(char byte, const char * msg){
    rosserial_msgs::Log l;
    l.level= byte;
    l.msg = (char*)msg;
    publish(rosserial_msgs::TopicInfo::ID_LOG, &l);
  }

public:
  void logdebug(const char* msg){
    log(rosserial_msgs::Log::ROSDEBUG, msg);
  }
  void loginfo(const char * msg){
    log(rosserial_msgs::Log::INFO, msg);
  }
  void logwarn(const char *msg){
    log(rosserial_msgs::Log::WARN, msg);
  }
  void logerror(const char*msg){
    log(rosserial_msgs::Log::ERROR, msg);
  }
  void logfatal(const char*msg){
    log(rosserial_msgs::Log::FATAL, msg);
  }

  /********************************************************************
       * Parameters
       */

private:
  bool param_recieved;
  rosserial_msgs::RequestParamResponse req_param_resp;

  bool requestParam(const char * name, int time_out =  1000){
    param_recieved = false;
    rosserial_msgs::RequestParamRequest req;
    req.name  = (char*)name;
    publish(TopicInfo::ID_PARAMETER_REQUEST, &req);
    uint32_t end_time = hardware_.time() + time_out;
    while(!param_recieved ){
      spinOnce();
      if (hardware_.time() > end_time) {
        logwarn("Failed to get param: timeout expired");
        return false;
      }
    }
    return true;
  }

public:
  bool getParam(const char* name, int* param, unsigned int length =1, int timeout = 1000){
    if (requestParam(name, timeout) ){
      if (length == req_param_resp.ints.size()){
        //copy it over
        for(size_t i=0; i<length; i++)
          param[i] = req_param_resp.ints[i];
        return true;
      } else {
        logwarn("Failed to get param: length mismatch");
      }
    }
    return false;
  }
  bool getParam(const char* name, float* param, unsigned int length=1, int timeout = 1000){
    if (requestParam(name, timeout) ){
      if (length == req_param_resp.floats.size()){
        //copy it over
        for(size_t i=0; i<length; i++)
          param[i] = req_param_resp.floats[i];
        return true;
      } else {
        logwarn("Failed to get param: length mismatch");
      }
    }
    return false;
  }
  bool getParam(const char* name, char** param, unsigned int length=1, int timeout = 1000){
    if (requestParam(name, timeout) ){
      if (length == req_param_resp.strings.size()){
        //copy it over
        for(size_t i=0; i<length; i++)
        {
          strcpy(param[i],req_param_resp.strings[i].c_str());
        }
        return true;
      } else {
        logwarn("Failed to get param: length mismatch");
      }
    }
    return false;
  }
};

}

#endif
