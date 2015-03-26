#!/usr/bin/env python

import communication
from observer import Observer
import rospy
from sonia_msgs.srv import vision_server_execute_cmd as service_ref

# Set the IP adress of the java socket server, localhost if on the same machine
TCP_IP = '127.0.0.1'
# Set the writing port of the socket |-> reading_port = port + 1
TCP_PORT = 46626
# Name of the current Node
NODE_NAME = 'auv6_communicator'
# ROS Service name
SERVICE_NAME = '/vision_server/vision_server_execute_cmd'


class ROSJavaCommunicator(Observer):

    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=False)
        self._topics = []

        self.ros_service_line = communication.ROSServiceCommunicationLine(
            SERVICE_NAME, service_ref)
        self.java_line = communication.JavaCommunicationLine(
            TCP_IP, TCP_PORT)

        self.java_line.attach(self.ros_service_line)
        self.ros_service_line.attach(self)


        rospy.spin()

    def _update(self, service):
        topic_name = service.recv()
        if topic_name.is_empty:
            return
        topic_name += "_result"

        print("Now listening " + topic_name)

        for topic in self._topics:
            if topic.get_name() == topic_name:
                rospy.logwarn(
                    "Sorry, but {!s} is already listening on topic {!s}"
                    .format(self.java_line.get_name(), topic.get_name()))
                return

        topic = communication.ROSTopicCommunicationLine(topic_name)
        topic.attach(self.java_line)
        self._topics.append(topic)

    def get_name(self):
        return "Control Loop"

    def send(self, data):
        """Send a message on the line
        Abstract method to rewrite
        """
        rospy.loginfo("I am supposed to send these datas to ROS service : \"" + data + "\"")
        topic_name = data.replace("response: ","")
        if topic_name == "''":
            return
        topic_name += "_result"

        print("Now listening " + topic_name)

        for topic in self._topics:
            if topic.get_name() == topic_name:
                rospy.logwarn(
                    "Sorry, but {!s} is already listening on topic {!s}, resetting topics"
                    .format(self.java_line.get_name(), topic.get_name()))
                self._topics.remove(topic)

        topic = communication.ROSTopicCommunicationLine(topic_name)
        topic.attach(self.java_line)
        self._topics.append(topic)


if __name__ == '__main__':
    ROSJavaCommunicator()
