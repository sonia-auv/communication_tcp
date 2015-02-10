#!/usr/bin/env python

from auv6_communicator.srv import Test
import rospy


def handle_test_service_server(req):
    print "I received a command"
    return "response string"


def test_service_server():
    rospy.init_node('test_service_server')
    rospy.Service('test_service_server', Test, handle_test_service_server)
    print "Ready to get an answer."
    rospy.spin()


if __name__ == "__main__":
    test_service_server()
