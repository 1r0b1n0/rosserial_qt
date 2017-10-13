#ifndef NODE_H
#define NODE_H

#include <QObject>
#include "ros/ros.h"
#include <QTimer>

#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>

typedef std_msgs::String MsgType;
static MsgType s_msg;

#define SEND_SIZE 50000

class Node : public QObject
{
    Q_OBJECT

public:
    explicit Node();

public slots:
    void onTimer();

private:
    ros::NodeHandle nh;
    QTimer *m_timer;
    ros::Publisher< MsgType > chatter;
    ros::Subscriber < MsgType > chatterSub;
};


#endif // NODE_H
