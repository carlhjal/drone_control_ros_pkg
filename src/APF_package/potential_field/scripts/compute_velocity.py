#! /usr/bin/env python3

import roslib, sys, rospy
from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32
import numpy as np
import math
from nav_msgs.msg import Odometry

#-------------------------------------------------

class safe_teleop:
    def __init__(self):
        rospy.init_node("command_velocity")

        rospy.Subscriber("potential_field_vector", Point, self.handle_potential_field)
        self.cmd_pub = rospy.Publisher("cmd_vel_robotont", Twist,queue_size=10)

        self.odom = None
        self.obstacle_vector = None
        self.min_vel_x = -1
        self.max_vel_x = 1
        self.min_vel_y = -1
        self.max_vel_y = 1
        self.drive_scale = 0.5 # scaling factor to scale the self.drive_scale = 0.1orce
        self.vx_1 = 0
        self.vy_1 = 0


#-------------------------------------------------

    def start(self):
        rate = rospy.Rate(rospy.get_param("~cmd_rate", 10))
        while not rospy.is_shutdown():
            cmd = self.compute_motion_cmd()
            if cmd != None:
                #print(f"publishing.. {cmd}")
                self.cmd_pub.publish(cmd)
            rate.sleep()

#-------------------------------------------------

    def compute_motion_cmd(self):
        if (self.obstacle_vector is None):
            cmd = None
        else:
            cmd = Twist()

            # We use velocity based potential field,that is, the gradient/force is directly commanded as velocities
            # instead of force or acceleration. 

            vel_x = self.obstacle_vector[0]*self.drive_scale
            vel_y = self.obstacle_vector[1]*self.drive_scale

            angle = np.arctan2(vel_y, vel_x)
            mag = np.sqrt(vel_y**2 + vel_x**2)

            vx = np.cos(angle)
            vy = np.sin(angle)
            vx = np.clip(vx, self.min_vel_x, self.max_vel_x) * mag
            vy = np.clip(vy, self.min_vel_y, self.max_vel_y) * mag

            vx = vx*0.8 + 0.2*self.vx_1
            vy = vy*0.8 + 0.2*self.vy_1

            self.vx_1 = vx
            self.vy_1 = vy

            cmd.linear.x = vx
            cmd.linear.y = vy

        return cmd

#-------------------------------------------------

    def handle_potential_field(self, potential_field):
        self.obstacle_vector = np.array([potential_field.x, potential_field.y])

if __name__ == "__main__":
    st = safe_teleop()
    st.start()