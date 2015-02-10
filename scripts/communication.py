#!/usr/bin/env python
"""Module for managing communication lines
Include class for Java and ROS communication
"""

import abc
from socket import socket
from threading import Thread
from std_msgs.msg import String
import sys
# ROS imports
import rospy

# Set a buffer max size for input from socket and output to ROS line
BUFFER_SIZE = 16


class Observable(object):
    """Simple Observer class
    Allow childs class to notify subscribers with notify
    """

    def __init__(self):
        """Default Constructor
        Initiat class attributes
        """
        self.observers = []

    def attach(self, observer):
        """Add a new observer to observers list
        """
        if observer not in self.observers:
            self.observers.append(observer)

    def detach(self, observer):
        """Remove an observer from observers list
        """
        try:
            self.observers.remove(observer)
        except ValueError:
            pass

    def notify(self, modifier=None):
        """Notify all observers that a change occurenced on self
        """
        if len(self.observers):
            for observer in self.observers:
                if modifier != observer:
                    observer.update(self)


class AbstractCommunicationLine(Observable, Thread):
    """Abstract methods and attributes for base communication lines
    This will provide a method send to send informations on the line
    and will run as a thread to get information from it
    """

    __metaclass__ = abc.ABCMeta  # ABC class behaves like abstract

    def __init__(self):
        """Default constructor, start connexions
        """
        self._input_stream = ''
        self._output_stream = ''
        self.started = False

        Thread.__init__(self)
        Observable.__init__(self)
        self.daemon = True

    def stop(self):
        """Stop communication line
        """
        self.started = False

    def send(self, data):
        """Send a message on the line
        Abstract method to rewrite
        """
        pass

    def recv(self):
        """Read ouput stream and empty it
        """
        tmp_input_stream = self._input_stream
        self._input_stream = ''
        return tmp_input_stream

    def run(self):
        """Method launched when object.start() is called on the instanciated
        object
        """
        pass

    def is_empty(self):
        """Check if the input stream is empty
        """
        if self._input_stream:
            return False
        return True

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
        self._client_ip = None
        self.port = port
        self._backlog = 5
        self._buffer_size = 1024
        self._server = None
        self._client = None

        self._connect()
        AbstractCommunicationLine.__init__(self)

    def _connect(self):
        """Connect to the client socket
        """
        try:
            self._server = socket()
            self._server.bind((self.host, self.port))
            self._server.listen(self._backlog)
        except:
            if self._server:
                self._server.close()
            rospy.logerr('Could not open socket')
            sys.exit(1)
        rospy.loginfo('Server running on port ' + str(self.port))

        self._client, self._client_ip = self._server.accept()
        rospy.loginfo('Connection from ' + self._client_ip[0])

    def stop(self):
        """Close connexion properly
        Override parent class to add socket closing process
        """
        socket.close()
        super(JavaCommunicationLine, self).stop()

    def _read_from_line(self):
        """Read informations from tcp socket
        """
        self._input_stream += str(self._client.recv(self._buffer_size))

    def send(self, data):
        """Send informations to tcp socket
        """
        rospy.loginfo(
            "I'm Sending data to Java : \"" +
            data +
            "\""
        )
        self._client.sendall(data)

    def run(self):
        """Method used by thread processing until stop() is used
        Will read on the line and notify observer if there is any informations
        """
        self.started = True
        while self.started:
            self._read_from_line()
            if not self.is_empty():
                rospy.loginfo(
                    "I received data from Java : \"" +
                    self._input_stream +
                    "\""
                )
                self.notify()

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Default Destructor
        Destroy the object correctly
        """
        self.stop()


class ROSTopicCommunicationLine(AbstractCommunicationLine):
    """Initiate a communication with ROS Topic given a writing
    and reading topic node_name
    """

    def __init__(self, node_name, writing_topic, reading_topic):
        """Default Constructor
        init node and topics
        """
        self._writing_topic = writing_topic
        self._reading_topic = reading_topic

        self.publisher = rospy.Publisher(
            self._writing_topic, String, queue_size=20)
        rospy.Subscriber(
            self._reading_topic, String, self._handle_read_subscribers)

        AbstractCommunicationLine.__init__(self)

    def send(self, data):
        """Send informations to publisher
        """
        self.publisher.publish(data)
        rospy.loginfo(
            "I'm sending data to ROS Topic : \"" +
            self._input_stream +
            "\""
        )

    def _handle_read_subscribers(self, data):
        """Method called when receiving informations from Subscribers
        """
        self.input_stream += data.data

    def run(self):
        """Method used by thread
        simply keeps python from exiting until this node is stopped
        """
        # TODO check if rospy.spin() is useful in this context
        while self.started and not rospy.is_shutdown():
            rospy.spin()


class ROSServiceCommunicationLine(AbstractCommunicationLine):
    """Initiate a communication with ROS given a service service_name
    """

    def __init__(self, _service_name, _service_ref):
        """Default constructor subscribe to service
        """
        self._service_name = _service_name
        self._service_ref = _service_ref

        self._service_response = rospy.ServiceProxy(
            self._service_name,
            self._service_ref
        )

        AbstractCommunicationLine.__init__(self)

    def send(self, data):
        """Loop and get information from service
        """
        rospy.wait_for_service(self._service_name)
        try:
            rospy.loginfo(
                "I'm sending data to ROS service : \"" +
                self._input_stream +
                "\""
            )
            self._input_stream += str(self._service_response(data))
            if not self.is_empty():
                rospy.loginfo(
                    "I received data from ROS service : \"" +
                    self._input_stream +
                    "\""
                )
                self.notify()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)

    def run(self):
        """Method used by thread
        simply keeps python from exiting until this node is stopped
        """
        self.started = True
        while self.started and not rospy.is_shutdown():
            rospy.spin()

    def update(self, subject):
        self.send(subject.recv())
