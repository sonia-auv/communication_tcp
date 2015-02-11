"""
"""

import abc


class Observable(object):
    """Simple Observer class
    Allow childs class to notify subscribers with notify
    """

    __metaclass__ = abc.ABCMeta  # ABC class behaves like abstract

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

    def _notify(self, modifier=None):
        """Notify all observers that a change occurenced on self
        """
        if len(self.observers):
            for observer in self.observers:
                if modifier != observer:
                    observer.update(self)


class Observer(object):
    """
    """

    __metaclass__ = abc.ABCMeta  # ABC class behaves like abstract

    def _update(self, subject):
        self.send(subject.recv())
