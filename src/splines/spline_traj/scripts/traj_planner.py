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
        
        self.q_star = 2
        self.obs_min_range = 0.02
        self.eta = 0.3

        self.odom = None
        self.laser = None
        
    def start(self):
        rate = rospy.Rate(self.sample_rate)
        while not rospy.is_shutdown():
            net_force = self.calc_next_goal()
            self.n_laser_scan(10)
            print(f"net_force before: {net_force}")
            net_force += self.obstacle_avoidance(net_force)
            print(f"net_force after: {net_force}")
            
            self.publish_sum(net_force[0],net_force[1])
            rate.sleep()


    def n_laser_scan(self, n: int) -> list:
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

        idx = np.argpartition(ranges, n)[:n]
        angles = resolution * idx
        distances = ranges[idx]
        #print(f"angles {angles}")
        #print(f"distances {distances}")
        
        n_closest = np.vstack((distances, angles))

        #print(f"n_closest {n_closest}")
        print(n_closest.shape)
        #n_closest = np.rot90(n_closest).tolist()
        return n_closest
    

###########################################################

    def compute_cbf(self, n_closest: np.ndarray):


        if self.odom == None:
            print("No target")
            return np.array([0,0])
        
        if np.isinf(n_closest).any():
            print("inf readings")
            return np.array([0,0])

        
        k_att = 1

        num_dim = 20

        odom_data = self.odom
        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y
        x_dot = odom_data.twist.twist.linear.x
        y_dot = odom_data.twist.twist.linear.y

        x_goal = x + x_dot*self.dt
        y_goal = y + y_dot*self.dt

        rel_x = []
        rel_y = []


        #print(f"n_closest recieved: {n_closest} and length: {len(n_closest)}")
        

        #print(f"distance: {distance}")
        #print(f"distance[0] {n_closest[0]}")

        distances = n_closest[0]


        mag = self.eta * (1 / self.q_star - 1 / n_closest[0]) / (n_closest[0]**2)
        
        print(f"mag {mag}")
        x_obs = mag * np.cos(n_closest[1])
        y_obs = mag * np.sin(n_closest[1])


        print(f"x_obs: {x_obs}")
        print(f"y_obs: {y_obs}")

        #x_obs = np.rot90(x_obs)
        #y_obs = np.rot90(y_obs)

        if x_obs.size == 0:
            return np.array([0,0])

        alpha = 0.6

        D_obs = 1.0

        norm_factor = np.sqrt( (x_obs)**2+(y_obs)**2   )

        dist_obs = np.sqrt( (x_obs)**2+(y_obs)**2   )-D_obs ## h(x)

        grad_hx = np.hstack(( (x_obs)/norm_factor,  (y_obs)/norm_factor)).reshape(1, 20)

        print(f"grad_hx {grad_hx.shape}")

        v_des = np.hstack(( -k_att*(x-x_goal), -k_att*(y-y_goal)))
        v_des = np.tile(v_des, (1,10)).reshape(1,20)

        print(f"v_des {v_des.shape}")

        Q = np.identity(num_dim)
        q = -v_des
        A_in = -grad_hx
        b_in = np.array( [alpha*dist_obs]  )
        print(f"shape: {np.shape(grad_hx)}")
        cvxopt.matrix(q, tc = 'd')
        cvxopt.matrix(Q, tc = 'd')
        cvxopt.matrix(A_in, tc = 'd')
        cvxopt.matrix(b_in, tc = 'd')
        #sol_data = solvers.qp( cvxopt.matrix(Q, tc = 'd'), cvxopt.matrix(q, tc = 'd'), cvxopt.matrix(A_in, tc = 'd'), cvxopt.matrix(b_in, tc = 'd'), None, None  )

        #sol = np.asarray(sol_data['x'])
        return np.array((0,0))#sol
    
    def compute_cbfa(self, n_closest: np.ndarray):

        #return np.array([0, 0])
        if self.odom == None:
            print("No target")
            return np.array([0, 0])

        if np.isinf(n_closest).any():
            print("inf readings")
            return np.array([0, 0])

        k_att = 1
        num_dim = 20

        odom_data = self.odom
        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y
        x_dot = odom_data.twist.twist.linear.x
        y_dot = odom_data.twist.twist.linear.y

        x_goal = x + x_dot * self.dt
        y_goal = y + y_dot * self.dt

        mag = self.eta * (1 / self.q_star - 1 / n_closest[0]) / (n_closest[0] ** 2)

        x_obs = mag * np.cos(n_closest[1])
        y_obs = mag * np.sin(n_closest[1])

        if x_obs.size == 0:
            return np.array([0, 0])

        alpha = 0.6
        D_obs = 1.0
        norm_factor = np.sqrt((x_obs) ** 2 + (y_obs) ** 2)
        dist_obs = np.sqrt((x_obs) ** 2 + (y_obs) ** 2) - D_obs

        # Set grad_hx to (2, 10) if that's the intended shape for A_in constraints
        grad_hx = np.vstack((x_obs / norm_factor, y_obs / norm_factor)).reshape(10, 2)
        A_in = - np.tile(grad_hx, (1, 10))

        # Reshape v_des and q to be compatible
        v_des = np.hstack((-k_att * (x - x_goal), -k_att * (y - y_goal)))
        q = -np.tile(v_des, 10).reshape(num_dim, 1)  # Ensure q is a column vector (20, 1)

        # Ensure Q is an identity matrix of shape (20, 20)
        Q = np.identity(num_dim)

        # Set A_in and b_in to be compatible with grad_hx
        #A_in = -grad_hx  # Shape: (2, 10) for A_in derived from grad_hx
        b_in = np.array([alpha * dist_obs]).reshape(10, 1)  # Ensure b_in is a column vector

        # Call the solver
        sol_data = solvers.qp(
            cvxopt.matrix(Q, tc='d'),
            cvxopt.matrix(q, tc='d'),
            cvxopt.matrix(A_in, tc='d'),
            cvxopt.matrix(b_in, tc='d'),
            None,
            None
        )

        sol = np.asarray(sol_data['x'])
        sol = np.array((sol[0,0], sol[1,0]))
        return sol
    
    def obstacle_avoidance(self, vel_vector: np.ndarray) -> np.ndarray:
        if self.odom == None or self.laser == None:
            print("No target")
            return np.array([0,0]) 

        n_closest = self.n_laser_scan(10)
        #print(f"n_closest from n_laser_scan that goes into computecbf {n_closest}")
        res = self.compute_cbf(n_closest)
        print(f"res: {res}") 

        


        return res *2
     
    
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

    def handle_laser(self, laser_data):
        self.laser = laser_data

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
