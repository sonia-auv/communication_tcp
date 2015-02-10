#!/usr/bin/env python

from auv6_communicator.srv import AddTwoInts
import rospy


def handle_add_two_ints(req):
    print "I received a command"
    return "response string"


def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
