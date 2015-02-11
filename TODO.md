Remove test config
==================

* In communication.py
  * change recv method to work with array
    recv must return data[0] and delete it
  * remove \n to messages from java
  * handle mutli connexions

* [DONE] In CMakeList.txt
  * [DONE] Remove AddTwoInts from add_service_files()
  * [DONE] Undo all the thing with message_generation
    see [http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv]
    comments 2 lines in package.xml aboute dependencies
