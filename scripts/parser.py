#!/usr/bin/env python

# ROS Imports
import rospy


def parse_from_java(str):
    parsed_tab = str.split(';')
    if len(parsed_tab) != 4:
        if len(parsed_tab) < 4:
            rospy.logerr(
                "The string from AUV6 does not contain enought arguments")
        if len(parsed_tab) > 4:
            rospy.logerr(
                "The string from AUV6 contains too much arguments")
        return None
    try:
        parsed_tab[3] = int(parsed_tab[3])
    except ValueError:
        rospy.logerr(
            "The fourth argument has to be an integer")
        return None
    return parsed_tab
