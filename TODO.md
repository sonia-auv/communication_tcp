Remove test config
==================

ROS Communicator
-------------------
  * [DONE] change recv method to work with array recv must return data[0] and delete it
  * [DONE] remove \n to messages from java
  * handle mutli connexions from auv6

Java TCP Line
-------------
  * Let's see if java can handle stop communication while reading on socket [@see Debug#Stop process]

Compilation
-----------------------
  * [DONE] Remove AddTwoInts from add_service_files()
  * [DONE] Undo all the thing with message_generation see [http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv] comments 2 lines in package.xml aboute dependencies

Debug
-----
### Launch process
  * Run Java then run AUV6 Communicator then run Vision Server
  * Run Java then run Vision Server then run AUV6 Communicator
  * Run AUV6 Communicator then run Java then run Vision Server
  * Run AUV6 Communicator then run Vision Server then run Java
  * Run Vision Server then run Java then run AUV6 Communicator
  * Run Vision Server then run AUV6 Communicator then run Java

### Communication content
  * [DONE] Send empty string
  * [DONE] Send none string (int, float, null)
  * [DONE] send string with wrong format

### Kill Process
  * Stop Java then reconnect it
  * [DONE] Stop Vision Server then reconnect it
  * Stop AUV6 Communicator then reconnect it

### Stop process
  * Stop AUV6 Communicator while reading on java
  * Stop Java while reading on AUV6 Communicator
