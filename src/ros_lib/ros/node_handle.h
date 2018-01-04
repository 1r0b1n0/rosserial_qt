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

#include <QObject>

#include <stdint.h>

#include "std_msgs/Time.h"
#include "rosserial_msgs/TopicInfo.h"
#include "rosserial_msgs/RequestParam.h"
#include "ros/msg.h"

namespace ros {

class NodeHandleBase_ : public QObject{
  Q_OBJECT
public:
  NodeHandleBase_(QObject *parent=0) : QObject(parent){}

  virtual int publish(int id, const Msg* msg)=0;
  virtual int spinOnce()=0;
  virtual bool connected() const=0;
};
}

#include "RosQtSocket.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/service_server.h"
#include "ros/service_client.h"
#include "ros/action_client.h"

namespace ros {

using rosserial_msgs::TopicInfo;

/* Node Handle */
class NodeHandle : public NodeHandleBase_
{
  Q_OBJECT  

  Q_PROPERTY(bool isConnected READ connected NOTIFY isConnectedChanged)
  Q_PROPERTY(float latency READ latency NOTIFY latencyChanged)

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
  NodeHandle(uint32_t input_size=10000, uint32_t output_size=10000, QObject *parent=0);

  void timerEvent(QTimerEvent *event) override;

  /* Start a named port, which may be network server IP, initialize buffers */
  void open(const std::string &hostName, uint16_t port = 11411);

  /* Disconnect from server */
  void close();

protected:
  //State machine variables for spinOnce
  int mode_;
  uint32_t bytes_;
  int topic_;
  int index_;
  unsigned int checksum_;

  bool configured_;
  bool isActive_; // true if we should try to connect
  float latency_;

  /* used for syncing the time */
  uint32_t last_sync_time;
  uint32_t last_sync_receive_time;
  uint32_t last_msg_timeout_time;

public:
  /* This function goes in your loop() function, it handles
   *  serial input and callbacks for subscribers.
   */


  virtual int spinOnce() override;


  /* Are we connected to the PC? */
  bool connected() const override;

  /********************************************************************
   * Time functions
   */

  void requestSyncTime();

  void syncTime(uint8_t * data);

  Time now();

  void setNow( Time & new_now );

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

  /* Register a new Action Client */
  template<typename ActionType>
  bool actionClient(ActionClient<ActionType>& ac){
    bool v = advertise(ac.goal_pub);
    v |= advertise(ac.cancel_pub);
    v |= subscribe(ac.status_sub);
    v |= subscribe(ac.feedback_sub);
    v |= subscribe(ac.result_sub);
    return v;
  }

  void negotiateTopics();

  virtual int publish(int id, const Msg * msg) override;

public slots:
  void setConfigured(bool isConnected);

private slots:
  void onReadyRead();

  /********************************************************************
   * Logging
   */

private:
  void log(char byte, const char * msg);

public:
  void logdebug(const char* msg);
  void loginfo(const char * msg);
  void logwarn(const char *msg);
  void logerror(const char*msg);
  void logfatal(const char*msg);

  /********************************************************************
   * Parameters
   */

private:
  bool param_recieved;
  rosserial_msgs::RequestParamResponse req_param_resp;

  bool requestParam(const std::string &name, int time_out =  1000);

public:
  bool getParam(const std::string &name, int* param, unsigned int length =1, int timeout = 1000);
  bool getParam(const std::string &name, float* param, unsigned int length=1, int timeout = 1000);
  bool getParam(const std::string &name, std::string *param, unsigned int length=1, int timeout = 1000);
  bool isConnected() const;
  float latency() const;

signals:
  void isConnectedChanged(bool isConnected);
  void latencyChanged(float latency);
};

}

#endif
