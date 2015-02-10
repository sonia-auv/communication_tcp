Remove test config
==================

* In CMakeList.txt
  * Remove AddTwoInts from add_service_files()
  * Undo all the thing with message_generation
    see [http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv]
    comments 2 lines in package.xml aboute dependencies
