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

#ifndef ROS_SERVICE_CLIENT_H_
#define ROS_SERVICE_CLIENT_H_

#include <functional>
#include "rosserial_msgs/ServiceCallResult.h"
#include "rosserial_msgs/TopicInfo.h"

#include "ros/publisher.h"
#include "ros/subscriber.h"

namespace ros {

  template<typename MType>
  class ServiceClient : public Subscriber_  {

    public:
      typedef typename MType::Request MReq;
      typedef typename MType::Response MRes;
      typedef std::function<void(const MRes&, bool)> CallbackT;
      typedef std::function<void(const MRes&, bool, uint8_t)> CallbackWithResultT;

      ServiceClient(const char* topic_name) :
        pub(topic_name, rosserial_msgs::TopicInfo::ID_SERVICE_CLIENT + rosserial_msgs::TopicInfo::ID_PUBLISHER)
      {
        this->topic_ = topic_name;
        this->waiting = false;
      }

      void call(const MReq& request, CallbackT callback)
      {
        if(!pub.nh_->connected()) return;
        waiting = true;
        cb_ = callback;
        pub.publish(request);
      }

      void call(const MReq& request, CallbackWithResultT callback)
      {
        if(!pub.nh_->connected()) return;
        waiting = true;
        cbWithResult_ = callback;
        pub.publish(request);
      }

      const std::string& getMsgType() override { return MRes::getType(); }
      const std::string& getMsgMD5() override { return MRes::getMD5(); }
      int getEndpointType() override{ return rosserial_msgs::TopicInfo::ID_SERVICE_CLIENT + rosserial_msgs::TopicInfo::ID_SUBSCRIBER; }

    private:
      // these refer to the subscriber
      void callback(unsigned char *data) override{
        MRes ret;
        // First byte give us the success status
        const uint8_t callResult = data[0];
        const bool success = callResult == rosserial_msgs::ServiceCallResult::SUCCESS;

        if(success)
        {
            // Access from the second byte
            ret.deserialize(data+1);
        }

        waiting = false;

        if(cb_)
        {
          cb_(ret, success);
        }
        else if(cbWithResult_)
        {
          cbWithResult_(ret, success, callResult);
        }

        cb_ = nullptr;
        cbWithResult_ = nullptr;
      }

      bool waiting;
      CallbackT cb_;
      CallbackWithResultT cbWithResult_;

    public:
      Publisher<MReq> pub;
  };

}

#endif
