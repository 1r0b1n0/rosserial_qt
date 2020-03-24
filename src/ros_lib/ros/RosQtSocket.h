/**
Software License Agreement (BSD)

\file      WindowsSocket.h
\authors   Kareem Shehata <kshehata@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ROS_QT_SOCKET_H_
#define ROS_QT_SOCKET_H_

#include <QObject>
#include "ros/RosQtSocket.h"
#include <string>
#include <iostream>
#include <QDateTime>
#include <QTcpSocket>
#include <QHostAddress>

class RosQtSocket : public QObject
{
  Q_OBJECT
public:

  RosQtSocket (QObject *parent=0);

  void doConnect();

  void open (const std::string &server_hostname, uint16_t port);

  void close ();

  int64_t read (unsigned char *data, int max_length);

  bool write (const unsigned char *data, int length);

  unsigned long time () const;

  QString getAddress() const;

private slots:
  void onConnected();

signals:
  void readyRead();

private:
  QTcpSocket socket_;
  bool socket_valid_;
  quint16 port_;
  QHostAddress address_;
};

#endif
