# rosserial_qt

Client implementation for rosserial using the multi-platform Qt library.
For the moment it only supports TCP/IP, but it could be easily adapted for UDP (using QUdpSocket) or serial port (using QSerialPort) communications.

## Usage

1. Generate the source files by running `rosrun rosserial_qt make_library.py <output_path>`
2. Compile your client application using the library. You should add the roslib library to you include path, and add the .cpp files to your source files.

Remarks:
* C++11 support is required
* Unlike other client implementations of rosserial, the *NodeHandle* uses a dynamically sized buffer.
* There is no need to call spinOnce() manually : the *NodeHandle* will call spinOnce() periodically and on reception of new data, using the Qt event loopŝ
* Unlike rosserial_arduino using C-style arrays, the message generation script will convert arrays to std::vector and std::array

## Examples
Some simple examples of applications using the library are in the *test* directory
To use the examples you should first generate the *ros_lib* directory, for example like this :
```
roscd rosserial_qt
rosrun rosserial_qt make_library.py .
```

## Performance
CPU usage seems to be quite close to native roscpp nodes.
(TODO : add some data)

## Supported platforms
This library should work on all platforms supported by Qt (Linux, Windows, Android, ios...)
I have succesfully tested this library on :
* Linux
* Windows

## Server-side configuration

The current implementation needs the message length to be coded on 4 bytes (rosserial's default is 2 bytes). This has been implemented in rosserial-server (modified version available here : [https://github.com/1r0b1n0/rosserial](https://github.com/1r0b1n0/rosserial) ), but has to be configured with the *msg_length_bytes* parameter.
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
