#include "ros/RosQtSocket.h"

//#define ROSQT_VERBOSE

using std::string;
using std::cerr;
using std::endl;


RosQtSocket::RosQtSocket(QObject *parent):
  QObject(parent),
  socket_valid_(false)
{
  QObject::connect(&socket_, SIGNAL(readyRead()), this, SIGNAL(readyRead()));
  QObject::connect(&socket_, SIGNAL(connected()), this, SLOT(onConnected()));
}

void RosQtSocket::doConnect()
{
  if(socket_.state() == QAbstractSocket::UnconnectedState)
  {
    socket_.connectToHost(address_, port_);
  }

}

void RosQtSocket::open(const std::string &server_hostname, uint16_t port)
{
  socket_valid_ = true;
  address_ = QHostAddress(QString::fromStdString(server_hostname));
  port_ = port;

  doConnect();
}

void RosQtSocket::close()
{
  socket_.close();
}

int64_t RosQtSocket::read(unsigned char *data, int max_length)
{
  if(socket_.state() != QAbstractSocket::ConnectedState)
  {
#ifdef ROSQT_VERBOSE
    std::cerr << "Failed to receive data from server, not connected " << std::endl;
#endif
    doConnect();
    return -1;
  }

  int64_t result = socket_.read((char*)data, max_length);
  if(result < 0)
  {
    std::cerr << "Failed to receive data from server" << std::endl;
    return -1;
  }

  return result;
}

bool RosQtSocket::write(const unsigned char *data, int length)
{
  if(socket_valid_ && socket_.state() != QAbstractSocket::ConnectedState)
  {
#ifdef ROSQT_VERBOSE
    std::cerr << "Failed to write data to the server, not connected " << std::endl;
#endif
    doConnect();
    return false;
  }
  else
  {
    qint64 result = socket_.write((const char*)data, length);
    if(result != length)
    {
      std::cerr << "Failed to write all the data to the server" << std::endl;
      return false;
    }
  }

  return true;
}

unsigned long RosQtSocket::time()
{
  return static_cast<unsigned long>(QDateTime::currentMSecsSinceEpoch());
}

QString RosQtSocket::getAddress() const
{
  return address_.toString();
}

void RosQtSocket::onConnected()
{
  socket_.setSocketOption(QAbstractSocket::LowDelayOption, 1);
}
