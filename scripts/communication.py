#!/usr/bin/env python
"""Module for managing communication lines
Include class for Java and ROS communication
"""

import abc
from socket import socket
from threading import Thread
import sys

from std_msgs.msg import String
from sonia_msgs.msg import filterchain_return_message as ret_str
import rospy
from observer import Observable, Observer
import parser


# Set a buffer max size for input from socket and output to ROS line
BUFFER_SIZE = 1024


class AbstractCommunicationLine(Observable, Observer, Thread):
    """Abstract methods and attributes for base communication lines
    This will provide a method send to send informations on the line
    and will run as a thread to get information from it
    """

    __metaclass__ = abc.ABCMeta  # ABC class behaves like abstract

    def __init__(self):
        """Default constructor, start connexions
        """
        Thread.__init__(self)
        Observable.__init__(self)
        Observer.__init__(self)

        self._input_stream = []
        self._output_stream = []
        self._connected = False
        self._running = False
        self.daemon = True

        self._connect()
        self.start()

    @abc.abstractmethod
    def _connect(self):
        """
        """
        raise NotImplementedError(
            "Class %s doesn't implement connect()" % self.__class__.__name__)

    @abc.abstractmethod
    def _process(self):
        """Method launched when object.start() is called on the instanciated
        object
        """
        raise NotImplementedError(
            "Class %s doesn't implement run()" % self.__class__.__name__)

    def run(self):
        """Method launched when object.start() is called on the instanciated
        object
        """
        self._running = True
        while self.is_running and not rospy.is_shutdown():
            self._process()
        self._running = False

    def recv(self):
        """Read ouput stream and empty it
        """
        tmp_input_stream = self._input_stream[0]
        self._input_stream.remove(self._input_stream[0])
        return tmp_input_stream

    def stop(self):
        """Stop communication line
        """
        self._running = False

    @abc.abstractmethod
    def send(self, data):
        """Send a message on the line
        Abstract method to rewrite
        """
        raise NotImplementedError(
            "Class %s doesn't implement send()" % self.__class__.__name__)

    @property
    def is_empty(self):
        """Check if the input stream is empty
        """
        if len(self._input_stream):
            return False
        return True

    @property
    def is_running(self):
        """Check if the input stream is empty
        """
        return self._running

    @property
    def is_connected(self):
        """Check if the input stream is empty
        """
        return self._connected

    @abc.abstractmethod
    def get_name(self):
        raise NotImplementedError

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Default Destructor
        Destroy the object correctly
        """
        self.stop()


class JavaCommunicationLine(AbstractCommunicationLine):
    """An easy to use API for making a dialog on TCP/IP Line
    This class is server class and provides reading and writing socket
    """

    def __init__(self, host='', port=46626):
        """Default constructor
        Initiate variables, Connect the socket and call parent constructor
        """
        self.host = host
        self.port = port
        self._backlog = 5
        self._socket = None
        self._clients = []
        self._started = False

        AbstractCommunicationLine.__init__(self)

    def _connect(self):
        """Connect to the client socket
        """
        try:
            self._socket = socket()
            self._socket.bind((self.host, self.port))
            self._socket.listen(self._backlog)
        except:
            if self._socket:
                self._socket.close()
            rospy.logerr('Could not open socket')
            sys.exit(1)
        rospy.loginfo(
            'Socket server running at : ' +
            str(self.host) + ":" + str(self.port))
        # Always accept connexions
        self._connexion_thread = Thread(target=self._accept_client)
        self._connexion_thread.daemon = True
        self._connexion_thread.start()

    def _accept_client(self):
        while True:
            client, client_ip = self._socket.accept()
            self._clients.append((client, client_ip))
            rospy.loginfo(
                'A client is connected : ' + str(self._clients[-1][1][0]) +
                ':' + str(self._clients[-1][1][1]))

    def stop(self):
        """Close connexion properly
        Override parent class to add socket closing process
        """
        self._socket.close()
        self._started = False
        super(JavaCommunicationLine, self).stop()

    def _process(self):
        """Method used by thread processing until stop() is used
        Will read on the line and notify observer if there is any informations
        """
        self._read_from_line()
        if not self.is_empty:
            self._notify()
        while len(self._output_stream):
            self._write_to_line()

    def _read_from_line(self):
        """Read informations from tcp socket
        """
        for client in self._clients:
            try:
                line = client[0].makefile().readline().rstrip('\n')
                if line:
                    rospy.loginfo("I received data from AUV6 : \"" + line + "\"")
                    if line == "END":
                        rospy.logwarn(
                            "The client {!s}:{!s} ended the connexion".format(
                                client[1][0], client[1][1]))
                        self._clients.remove(client)
                        return
                    self._input_stream.append(line)
            except:
                rospy.logerr(
                    "The client {!s}:{!s} disconnected without closing the "
                    .format(client[1][0], client[1][1]) + "connexion")
                self._clients.remove(client)

    def _write_to_line(self):
        for client in self._clients:
            rospy.loginfo(
                "I am Sending data to AUV6 on {!s}:{!s} : \"".format(
                    client[1][0], client[1][1]) +
                self._output_stream[0] + "\"")
            try:
                client[0].send(self._output_stream[0] + "\n")
            except:
                rospy.logerr(
                    "The client {!s}:{!s} disconnected without "
                    .format(client[1][0], client[1][1]) +
                    "closing the connexion")
                self._clients.remove(client)
        self._output_stream = self._output_stream[1:]

    def send(self, data):
        """Send informations to tcp socket
        """
        for client in self._clients:
            rospy.loginfo(
                "I am Sending data to AUV6 on {!s}:{!s} : \"".format(
                    client[1][0], client[1][1]) +
                data + "\"")
            try:
                client[0].send(data + "\n")
            except:
                rospy.logerr(
                    "The client {!s}:{!s} disconnected without "
                    .format(client[1][0], client[1][1]) +
                    "closing the connexion")
                self._clients.remove(client)

    def get_name(self):
        return "AUV6"

    def update(self, subject):
        rospy.loginfo(
                "I am Sending data to AUV6 :::")
        self.send(subject.recv())


class ROSTopicCommunicationLine(AbstractCommunicationLine):
    """Initiate a communication with ROS Topic given a writing
    and reading topic node_name
    """

    def __init__(self, reading_topic, writing_topic=None):
        """Default Constructor
        init node and topics
        """
        self._writing_topic = writing_topic
        self._reading_topic = reading_topic

        AbstractCommunicationLine.__init__(self)

    def _connect(self):
        """
        """
        rospy.loginfo(
            "I am subscribing to ROS reading topic : " + self._reading_topic)
        rospy.Subscriber(
            self._reading_topic, ret_str, self._handle_read_subscribers)
        if self._writing_topic:
            rospy.loginfo(
                "I am subscribing to ROS writing topic : " +
                self.writing_topic)
            self.publisher = rospy.Publisher(
                self._writing_topic, String, queue_size=20)

    def _process(self):
        """Method used by thread
        simply keeps python from exiting until this node is stopped
        """
        if len(self._output_stream):
            if self._writing_topic:
                rospy.loginfo(
                    "I am sending data to ROS Topic : \"" +
                    self._output_stream[0] + "\"")
                self.publisher.publish(self._output_stream[0])
                self._output_stream = self._output_stream[1:]
            else:
                rospy.logerr(
                    "Sorry, you did not provide me any topic to publish on...")

    def _handle_read_subscribers(self, data):
        """Method called when receiving informations from Subscribers
        """
        self._input_stream.append(data.execution_result)
        rospy.loginfo(
            "I received data from ROS Topic : \"" +
            self._input_stream[-1] +
            "\" - ["+ data.execution_result +"] "
        )
        self._notify()

    def send(self, data):
        """Send informations to publisher
        """
        self._output_stream.append(data)

    def get_name(self):
        return self._reading_topic


class ROSServiceCommunicationLine(AbstractCommunicationLine):
    """Initiate a communication with ROS given a service service_name
    """

    def __init__(self, service_name, service_ref):
        """Default constructor subscribe to service
        """
        self._service_name = service_name
        self._service_ref = service_ref

        AbstractCommunicationLine.__init__(self)

    def _connect(self):
        """
        """
        rospy.loginfo("I am connecting to Vision Server ROS service")
        self._service_response = rospy.ServiceProxy(
            self._service_name, self._service_ref)

    def _process(self):
        """Method used by thread
        simply keeps python from exiting until this node is stopped
        """
        if len(self._output_stream):
            rospy.wait_for_service(self._service_name)
            try:
                rospy.loginfo(
                    "I am sending data to Vision Server : \"" +
                    "node_name : " + self._output_stream[0][0] +
                    " filterchain_name : " + self._output_stream[0][1] +
                    " media_name : " + self._output_stream[0][2] +
                    " cmd : " + str(self._output_stream[0][3]) + "\"")
                self._input_stream.append(str(self._service_response(
                    self._output_stream[0][0], self._output_stream[0][1],
                    self._output_stream[0][2], self._output_stream[0][3])))
                self._output_stream = self._output_stream[1:]
                if not self.is_empty:
                    rospy.loginfo(
                        "I received data from Vision Server : \"" +
                        self._input_stream[-1] + "\"")
                    self._notify()
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s" % e)

    def send(self, node_name, filterchain_name, media_name, cmd):
        """Loop and get information from service
        """
        self._output_stream.append((
            node_name, filterchain_name, media_name, cmd))

    def update(self, subject):
        """
        """
        parsed_str = parser.parse_from_java(subject.recv())
        if parsed_str:
            self.send(
                parsed_str[0], parsed_str[1], parsed_str[2], parsed_str[3])

    def get_name(self):
        return self._service_name
