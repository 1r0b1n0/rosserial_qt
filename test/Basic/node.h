#ifndef NODE_H
#define NODE_H

#include <QObject>
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <QTimer>
#include <roscpp/GetLoggers.h>
#include <roscpp_tutorials/TwoInts.h>

class Node : public QObject
{
    Q_OBJECT

public:
    explicit Node();

public slots:
    void onTimer();

private:
    void addTwoInts(const roscpp_tutorials::TwoIntsRequest &req, roscpp_tutorials::TwoIntsResponse &res);

    ros::NodeHandle nh;
    QTimer *m_timer;
    ros::Publisher< std_msgs::String > chatter;
    ros::Subscriber < geometry_msgs::PoseWithCovarianceStamped > poseSub;
    ros::ServiceClient<roscpp::GetLoggers> serviceClient;
    ros::ServiceServer<roscpp_tutorials::TwoInts> serviceServer;
};


#endif // NODE_H
