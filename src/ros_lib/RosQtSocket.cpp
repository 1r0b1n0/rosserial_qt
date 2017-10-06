#include "ros/RosQtSocket.h"

#define DEFAULT_PORT 11411

using std::string;
using std::cerr;
using std::endl;


RosQtSocket::RosQtSocket(QObject *parent):
  QObject(parent)
{
  QObject::connect(&m_socket, SIGNAL(readyRead()), this, SIGNAL(readyRead()));
  QObject::connect(&m_socket, SIGNAL(connected()), this, SLOT(onConnected()));
}

void RosQtSocket::doConnect()
{
  if(m_socket.state() == QAbstractSocket::UnconnectedState)
  {
    m_socket.connectToHost(m_address, m_port);
  }

}

void RosQtSocket::init(const std::string &server_hostname)
{
  m_address = QHostAddress(QString::fromStdString(server_hostname));
  m_port = DEFAULT_PORT;

  doConnect();
}

int64_t RosQtSocket::read(unsigned char *data, int max_length)
{
  if(m_socket.state() != QAbstractSocket::ConnectedState)
  {
    std::cerr << "Failed to receive data from server, not connected " << std::endl;
    doConnect();
    return -1;
  }

  int64_t result = m_socket.read((char*)data, max_length);
  if(result < 0)
  {
    std::cerr << "Failed to receive data from server" << std::endl;
    return -1;
  }

  return result;
}

bool RosQtSocket::write(const unsigned char *data, int length)
{
  if(m_socket.state() != QAbstractSocket::ConnectedState)
  {
    std::cerr << "Failed to write data to the server, not connected " << std::endl;
    doConnect();
    return false;
  }
  else
  {
    qint64 result = m_socket.write((const char*)data, length);
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

void RosQtSocket::onConnected()
{
  m_socket.setSocketOption(QAbstractSocket::LowDelayOption, 1);
}
