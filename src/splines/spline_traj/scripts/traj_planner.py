#! /usr/bin/env python3

from visualization_msgs.msg import Marker,MarkerArray
import roslib, sys, rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import Point, Twist, PoseStamped
import numpy as np
import os.path
import spline_gen as spline
import cvxopt
from cvxopt import solvers

class Spline:
    def __init__(self, path: np.ndarray):
        """
        Contains:

        xd = cs_x_path
        
        yd = cs_y_path
        
        xd'= x_d_dot_path
        
        yd' = y_d_dot_path

        and other shit in unusable format
        """
        self.x_path = path[0]
        self.y_path = path[1]
        (self.cs_x_path, 
         self.cs_y_path, 
         self.x_d_dot_path, 
         self.y_d_dot_path, 
         self.cs_phi_path, 
         self.arc_length, 
         self.arc_vec) = spline.path_spline(self.x_path, self.y_path)

class Waypoints:
    def __init__(self, spline_obj: Spline, x_global_init, y_global_init, v, dt):
        """
        Contains:

        xd = cs_x_path
        
        yd = cs_y_path
        
        xd'= x_d_dot_path
        
        yd' = y_d_dot_path

        and other shit in usable format
        """
        #x_global_init = 0
        #y_global_init = 0
        (self.x_waypoints, 
         self.y_waypoints, 
         self.phi_Waypoints, 
         self.xd_dot, 
         self.yd_dot, 
         self.x_d, 
         self.y_d) =  spline.waypoint_generator(x_global_init,
                                                y_global_init, 
                                                spline_obj.x_path, 
                                                spline_obj.y_path,
                                                spline_obj.arc_vec,
                                                spline_obj.cs_x_path,
                                                spline_obj.cs_y_path,
                                                spline_obj.cs_phi_path,
                                                spline_obj.arc_length,
                                                spline_obj.x_d_dot_path,
                                                spline_obj.y_d_dot_path,
                                                v,
                                                dt)
        
        

class Traj_planner:
    def __init__(self, trajectory: np.ndarray):
        self.v = 0.5
        self.dt = 0.1

        self.s = Spline(get_trajectory(trajectory))
        self.waypoints = Waypoints(self.s, x_global_init=0, y_global_init=0, v=self.v, dt=self.dt)

        rospy.init_node("trajectory_planner")
        self.sample_rate = rospy.get_param("~sample_rate", 10)

        # Subscribe to the global planner using the move base package. The global plan is the path that the robot would ideally follow if 
        # there are no unknown/dynamic obstacles. In the videos this is highlighted by green color.
        #self.global_path_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.handle_global_path)

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.handle_laser)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.handle_odom)
        self.cmd_pub = rospy.Publisher("cmd_vel_robotont", Twist,queue_size=10)
        self.global_path_pub = rospy.Publisher("global_path",Path,queue_size=10)

        #This is a publisher to publish the robot path. In the videos it is highlighted by red color.
        self.robot_path_pub = rospy.Publisher("robot_path",Path,queue_size=10)

        #self.cmd_pub = rospy.Publisher("cmd_vel_robotont", Twist,queue_size=10)
        self.v = 1
        self.dt = 0.1
        self.kp = 0.5
        self.kd = 0.5
        
        self.q_star = 0.4
        self.obs_min_range = 0.02

        self.odom = None
        self.laser = None
        
    def start(self):
        rate = rospy.Rate(self.sample_rate)
        while not rospy.is_shutdown():
            net_force = self.calc_next_goal()
            self.n_laser_scan(10)
            #net_force = self.obstacle_avoidance(net_force)

            self.publish_sum(net_force[0],net_force[1])
            rate.sleep()


    def n_laser_scan(self, n: int) -> np.ndarray:
        if self.laser == None or self.odom == None:
            return np.array([0.,0.])
    
        laser_data = self.laser
        ranges = np.array(laser_data.ranges)
        angle = laser_data.angle_min
        resolution = laser_data.angle_increment
        vector_sum = np.array([0.0,0.0])

        # The lidar outputs 360 degree data and therefore we get data of 360 points within the range of the sensor.
        # Each obstacle detected by the sensor will occupy some points out of these 360 points. We treat each single point as 
        # an individual obstacle.

        readings = np.array((370, 2))
        idx = np.argpartition(ranges, n)
        n_closest = np.array([readings[idx],angle*idx])
        print(n_closest)
        # Normalization of the repulsive forces
    

###########################################################

    def compute_cbf(xo, yo, x_obs, y_obs, D_obs, x_goal, y_goal):
        k_att = 1

        num_dim = 2

        x = xo
        y = yo

        alpha = 0.6

        D_obs = 1.0

        norm_factor = np.sqrt( (x-x_obs)**2+(y-y_obs)**2   )

        dist_obs = np.sqrt( (x-x_obs)**2+(y-y_obs)**2   )-D_obs ## h(x)

        grad_hx = np.hstack(( (x-x_obs)/norm_factor,  (y-y_obs)/norm_factor)).reshape(1, num_dim)

        v_des = np.hstack(( -k_att*(x-x_goal), -k_att*(y-y_goal)))

        Q = np.identity(num_dim)
        q = -v_des
        A_in = -grad_hx
        b_in = np.array( [alpha*dist_obs]  )
        # print(np.shape(grad_hx))

        sol_data = solvers.qp( cvxopt.matrix(Q, tc = 'd'), cvxopt.matrix(q, tc = 'd'), cvxopt.matrix(A_in, tc = 'd'), cvxopt.matrix(b_in, tc = 'd'), None, None  )

        sol = np.asarray(sol_data['x'])
        return sol
    
    def obstacle_avoidance(self, vel_vector: np.ndarray) -> np.ndarray:
        if self.odom == None:
            print("No target")
            return np.array([0,0])            
        


        return np.array([vx, vy])
     
    
    def calc_next_goal(self) -> np.ndarray:
        if self.odom == None:
            print("No target")
            return np.array([0,0])
        
        odom_data = self.odom
        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y
        x_dot = odom_data.twist.twist.linear.x
        y_dot = odom_data.twist.twist.linear.y

        self.waypoints = Waypoints(self.s, x_global_init=x, y_global_init=y, v=self.v, dt=self.dt)
        
        vx = self.kp*(self.waypoints.x_d - x) + self.kd*(self.waypoints.xd_dot - x_dot)
        vy = self.kp*(self.waypoints.y_d - y) + self.kd*(self.waypoints.yd_dot - y_dot)

        return np.array([vx, vy])
    
        #self.average_movement_random_motion = np.roll(self.average_movement_random_motion, 1, axis=0)
    
    def publish_sum(self, vx, vy):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        self.cmd_pub.publish(cmd)

#--------------------------------------------------------#
#                     helper functions                   #
#--------------------------------------------------------#

    def handle_odom(self, odom_data):
        self.odom = odom_data

def get_trajectory(trajectory_name: str) -> np.ndarray:
    """
    loads a .npy trajectory and returns a 2D numpy array
    """
    resource_path = os.path.join(os.path.dirname(os.path.realpath(__file__)))
    resource_path = resource_path.replace("scripts", "resources")
    file =  os.path.join(resource_path, trajectory_name)
    file = os.path.relpath(file)
    return np.load(file)

if __name__ == "__main__":
    
    tp = Traj_planner("circle.npy")
    tp.start()