"""
"""

import abc
# ROS Imports
import rospy


class Observable(object):
    """Simple Observer class
    Allow childs class to notify subscribers with notify
    """

    __metaclass__ = abc.ABCMeta  # ABC class behaves like abstract

    def __init__(self):
        """Default Constructor
        Initiat class attributes
        """
        self._observers = []

    def attach(self, observer):
        """Add a new observer to observers list
        """
        if observer not in self._observers:
            self._observers.append(observer)
            rospy.loginfo(
                "{!s} is now listening {!s}".format(
                    observer.get_name(), self.get_name()))
        else:
            rospy.logwarn(
                "Try to attach {!s} to {!s}, but it was already listening"
                .format(observer.get_name(), self.get_name()))

    def detach(self, observer):
        """Remove an observer from observers list
        """
        try:
            self._observers.remove(observer)
        except ValueError:
            pass

    def _notify(self, modifier=None):
        """Notify all observers that a change occurenced on self
        """
        if len(self._observers):
            for observer in self._observers:
                if modifier != observer:
                    observer._update(self)


class Observer(object):
    """
    """

    __metaclass__ = abc.ABCMeta  # ABC class behaves like abstract

    @abc.abstractmethod
    def get_name(self):
        pass

    def _update(self, subject):
        self.send(subject.recv())
