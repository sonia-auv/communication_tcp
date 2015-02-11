#!/usr/bin/env python

import communication
# ROS imports
import rospy
from sonia_msgs.srv import vision_server_execute_cmd as SERVICE_REF

# Set the IP adress of the java socket server, localhost if on the same machine
TCP_IP = '127.0.0.1'
# Set the writing port of the socket |-> reading_port = port + 1
TCP_PORT = 46626
# Name of the current Node
NODE_NAME = 'auv6_communicator'
# ROS Service name
SERVICE_NAME = '/vision_server/vision_server_execute_cmd'


class ROSJavaCommunicator(object):

    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=False)
        self.topics = []

        self.ros_service_line = communication.ROSServiceCommunicationLine(
            SERVICE_NAME, SERVICE_REF)
        self.java_line = communication.JavaCommunicationLine(
            TCP_IP, TCP_PORT)

        self.java_line.attach(self.ros_service_line)
        self.ros_service_line.attach(self)

        self.java_line.start()
        self.ros_service_line.start()

        rospy.spin()

    def run(self):
        topic = communication.ROSTopicCommunicationLine('test_talker')
        self.topics.append(topic)
        topic.attach(self)
        # while True:
        #    self.ros_service_line.send("test", "test", "Webcam", 1)

    def update(self, service):
        topic = communication.ROSTopicCommunicationLine('test_talker')
        self.topics.append(topic)
        topic.attach(self.java_line)
        topic.start()


if __name__ == '__main__':
    ROSJavaCommunicator()
