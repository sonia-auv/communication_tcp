#!/usr/bin/env python

import communication
# ROS imports
import rospy
from vision_server.srv import vision_server_execute_cmd as SERVICE_REF
import parser

# Set the IP adress of the java socket server, localhost if on the same machine
TCP_IP = '127.0.0.1'
# Set the writing port of the socket |-> reading_port = port + 1
TCP_PORT = 46626
# Name of this current Node
NODE_NAME = 'test_auv6_communicator'
# ROS Service name
SERVICE_NAME = 'vision_server_execute_cmd'


class ROSJavaCommunicator(object):

    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=False)
        self.topics = []

        self.java_line = communication.JavaCommunicationLine(
            TCP_IP, TCP_PORT)
        self.ros_service_line = communication.ROSServiceCommunicationLine(
            SERVICE_NAME, SERVICE_REF)

        self.java_line.attach(self.ros_service_line)
        self.ros_service_line.attach(self)

        self.java_line.start()
        self.ros_service_line.start()

        self.run()

        rospy.spin()

    def run(self):
        while True:
            self.ros_service_line.send("toto")

    def update(self, service):
        response = service.recv()
        print(response)
        # parsed_response = parser.parse_service_response(response)
        # topic = communication.ROSTopicCommunicationLine(
        #     parsed_response[0], parsed_response[1])
        # topic.attach(self.java_line)
        # self.topics.append(topic)


if __name__ == '__main__':
    ROSJavaCommunicator()
