#!/usr/bin/env python

import communications
import sys
# ROS imports
import rospy
from vision_server.srv import command as SERVICE_REF

# Set the IP adress of the java socket server, localhost if on the same machine
TCP_IP = '127.0.0.1'
# Set the writing port of the socket |-> reading_port = port + 1
TCP_PORT = 46626
# Name of this current Node
NODE_NAME = 'auv6_communicator'
# ROS Service name
SERVICE_NAME = 'add_two_ints'


class ROSJavaCommunicator(object):

    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=False)

        # self.java_line = communications.JavaCommunicationLine(
        #     TCP_IP, TCP_PORT)
        # self.java_line.attach(self)
        # self.java_line.start()

        # self.ros_topic_line = communications.ROSTopicCommunicationLine(
        #     '', '', '')

        self.ros_service_line = communications.ROSServiceCommunicationLine(
            SERVICE_NAME, SERVICE_REF)
        self.ros_service_line.start()

        self.run()

    def run(self):
        i = 0
        while True:
            i += 1
            # self.java_line.send("This is a message from python num " + str(i))
            self.ros_service_line.send("This is a message from python num " + str(i))

    def update(self, subject):
        sys.stdout.write(subject.recv())

if __name__ == '__main__':
    ROSJavaCommunicator()
