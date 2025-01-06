#! /usr/bin/env python3

import rospy
from std_msgs.msg import Empty
import numpy as np

class Commandeer:
    def __init__(self):

        rospy.init_node("commandeer_node")
        
        command = rospy.get_param("~command", "reset")
        
        valid = ["takeoff", "land", "reset"]
        if command not in valid:
            rospy.signal_shutdown(f"Invalid command: {command}")
            return
        
        topic = "bebop/" + command
        print(topic)
        cmd_pub = rospy.Publisher(topic, Empty, queue_size=10)
        rospy.sleep(0.5)

        cmd_pub.publish(Empty())
        rospy.signal_shutdown(f"Published empty message to: {topic}")

if __name__ == "__main__":
    cmd = Commandeer()

