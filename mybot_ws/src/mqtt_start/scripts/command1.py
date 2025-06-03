#!/usr/bin/env python

import rospy
from utils.srv import CommandLong


def handle_command(req):
    rospy.loginfo("Received command: %d", req.command)
    success = True
    result = 1
    return success, result

def command_server():
    rospy.init_node('command_server')
    rospy.Service('command/start_mission', CommandLong, handle_command)
    rospy.loginfo("Command server ready.")
    rospy.spin()

if __name__ == "__main__":
    command_server()