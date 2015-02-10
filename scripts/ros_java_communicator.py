#!/usr/bin/env python

import communication
# ROS imports
import rospy
from auv6_communicator.srv import Test as SERVICE_REF

# Set the IP adress of the java socket server, localhost if on the same machine
TCP_IP = '127.0.0.1'
# Set the writing port of the socket |-> reading_port = port + 1
TCP_PORT = 46626
# Name of this current Node
NODE_NAME = 'auv6_communicator'
# ROS Service name
SERVICE_NAME = 'test_service_server'


class ROSJavaCommunicator(object):

    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=False)

        self.java_line = communication.JavaCommunicationLine(
            TCP_IP, TCP_PORT)
        self.ros_service_line = communication.ROSServiceCommunicationLine(
            SERVICE_NAME, SERVICE_REF)
        # self.ros_topic_line = communications.ROSTopicCommunicationLine(
        #     '', '', '')

        self.java_line.attach(self.ros_service_line)

        self.java_line.start()
        self.ros_service_line.start()

        rospy.spin()


if __name__ == '__main__':
    ROSJavaCommunicator()
