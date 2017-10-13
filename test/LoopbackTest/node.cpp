#include "node.h"

void chatter_callback (const MsgType &msg)
{
  if(msg.data == s_msg.data)
  {
    //std::cout << "received correct data "<< std::endl;
  }
  else
  {
    std::cout << "received incorrect data "<< std::endl;
  }
  //std::cout << "received : " << msg.data << std::endl;
}

Node::Node():
  chatter("chatter"),
  chatterSub("chatter_out", &chatter_callback)
{
  std::string ros_master = "127.0.0.1";
  s_msg.data.resize(SEND_SIZE);
  for(size_t i=0;i<s_msg.data.size();++i)
  {
    s_msg.data[i] = i%256;
  }

  printf ("Connecting to server at %s\n", ros_master.c_str());
  nh.initNode (ros_master.c_str());

  nh.subscribe (chatterSub);
  nh.advertise(chatter);

  m_timer = new QTimer();
  connect(m_timer, SIGNAL(timeout()), this, SLOT(onTimer()));
  m_timer->start(10);
}

void Node::onTimer()
{
  static int i=1;
  i++;

  //std::cout << "Sending string with " << s_msg.data.size() << " bytes of data" << std::endl;
  for(int i=0;i<10;i++)
  {
    chatter.publish(s_msg);
  }
}

