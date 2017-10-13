# rosserial_qt

Client implementation for rosserial using the multi-platform Qt library.
For the moment it only supports TCP/IP, but it should be easily adapted to UDP (using QUdpSocket) or serial port (using QSerialPort) communications.

## Usage

1. Generate the source files by running `rosrun rosserial_qt make_library.py <output_path>`
2. Compile your client application using the library. You should add the roslib library to you include path, and add the .cpp files to your source files.

Remarks:
* Unlike other clients of rosserial, the *NodeHandle* uses a dynamically sized buffer.
* There is no need to call spinOnce() manually : the *NodeHandle* will call spinOnce() periodically and on reception of new data
* Unlike rosserial_arduino using C-style arrays, the message generation script will convert arrays to std::vector and std::array

## Examples
Some simple examples of applications using the library are in the *test* directory

## Performance
CPU usage is quite close to native roscpp nodes :

## Supported platforms
This library should work on all platforms supported by Qt (Linux, Windows, macos, ios, android ...)
I have succesfully tested this library on :
* Linux
* Windows

## Server-side configuration

The current implementation needs the message length to be coded on 4 bytes (rosserial's default is 2 bytes). This has been implemented in rosserial-server, but has to be configured with the *msg_length_bytes* parameter.
You should also configure the receive buffer size depending of your application.
The rosserial-python server implementation will not work, as it doesn't support 4 bytes coding of the message length.

Example launch file :
```
<launch>
  <node pkg="rosserial_server" type="socket_node" name="rosserial_server_socket_node">
    <param name="msg_length_bytes" value="4"/>
    <param name="buffer_size" value="1000000"/>
    <param name="tcp_nodelay" value="true"/>
  </node>
  <node pkg="rosserial_python" type="message_info_service.py" name="rosserial_message_info" />
</launch>
```
