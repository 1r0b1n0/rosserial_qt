#include "node.h"
#include <roscpp_tutorials/TwoInts.h>
#include <rosserial_msgs/ServiceCallResult.h>

Node::Node():
  chatter("chatter"),
  poseSub("estimated_pose", std::bind(&Node::estimated_pose_callback, this, std::placeholders::_1)),
  serviceClient("/rosout/get_loggers"),
  serviceServer("add_two_ints", std::bind(&Node::addTwoInts, this, std::placeholders::_1, std::placeholders::_2))
{
  std::string ros_master = "127.0.0.1";

  printf ("Connecting to server at %s\n", ros_master.c_str());
  nh.open (ros_master.c_str());

  nh.subscribe (poseSub);
  nh.advertise(chatter);

  m_timer = new QTimer();
  connect(m_timer, SIGNAL(timeout()), this, SLOT(onTimer()));
  m_timer->start(1000);

  nh.serviceClient(serviceClient);

  // not yet supported by rosserial_server
  //nh.advertiseService(serviceServer);
}

void Node::estimated_pose_callback (const geometry_msgs::PoseWithCovarianceStamped & pose)
{
  printf ("Received pose %f, %f, %f\n", pose.pose.pose.position.x,
          pose.pose.pose.position.y, pose.pose.pose.position.z);

  std::cout << "Received pose : " << QJsonDocument(pose.serializeAsJson()).toJson(QJsonDocument::Indented).toStdString() << std::endl;
}


void Node::onTimer()
{
  static int i=1;
  i++;
  std_msgs::String str_msg;
  str_msg.data = QString("Hello %1").arg(i).toStdString();
  chatter.publish(str_msg);

  roscpp::GetLoggersRequest req;
  serviceClient.call(req, [this](const roscpp::GetLoggers::Response &loggers, bool ok, uint8_t callResult){
    switch(callResult)
    {
    case rosserial_msgs::ServiceCallResult::NO_EXISTENCE:
        std::cout << "Service " << serviceClient.topic_ << " does not exist" << std::endl;
        break;
    case rosserial_msgs::ServiceCallResult::CALL_FAILED:
        std::cout << "Call to service " << serviceClient.topic_ << " failed" << std::endl;
        break;
    case rosserial_msgs::ServiceCallResult::SUCCESS:
      std::cout << "loggers : " << QJsonDocument(loggers.serializeAsJson()).toJson(QJsonDocument::Indented).toStdString() << std::endl;
      break;
    }
  });
}

void Node::addTwoInts(const roscpp_tutorials::TwoIntsRequest &req, roscpp_tutorials::TwoIntsResponse &res)
{
  std::cout << "addTwoInts" << std::endl;
  res.sum = req.a + req.b;
}

