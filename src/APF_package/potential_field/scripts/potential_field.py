#! /usr/bin/env python3

from cmath import inf
from visualization_msgs.msg import Marker,MarkerArray
import roslib, sys, rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import Point,PoseStamped
import numpy as np
import math
import time
#------------------------------------------

class potential_field:
    def __init__(self):
        rospy.init_node("potential_field")

        self.sample_rate = rospy.get_param("~sample_rate", 10)

        # Subscribe to the global planner using the move base package. The global plan is the path that the robot would ideally follow if 
        # there are no unknown/dynamic obstacles. In the videos this is highlighted by green color.
        self.global_path_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.handle_global_path)

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.handle_laser)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.handle_odom)
        
        # Subscribe to the goal topic to get the goal position given using rviz's 2D Navigation Goal option.
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.handle_goal)

        # Publish the potential field vector topic which will be subscribed by the command_velocity node in order to
        # compute velocities.
        self.potential_field_pub = rospy.Publisher("potential_field_vector", Point,queue_size=10)

        # We store the path data gotten from the global planner above and display it. We have written a custom publisher 
        # in order to get more flexibility while displaying the paths.
        self.global_path_pub = rospy.Publisher("global_path",Path,queue_size=10)

        # This is a publisher to publish the robot path. In the videos it is highlighted by red color.
        self.robot_path_pub = rospy.Publisher("robot_path",Path,queue_size=10)
        
        self.path_robot = Path()
        self.path_robot.header.frame_id = 'map'

        ## TODO Choose suitable values
        self.eta = 0.5 # scaling factor for repulsive force
        self.zeta = 0.5 # scaling factor for attractive force
        self.q_star = 3 # threshold distance for obstacles
        self.d_star = 1.5 # threshoild distance for goal

        self.laser = None
        self.odom = None
        self.goal = None

        self.path_data = Path()
        self.path_data.header.frame_id = 'map'
        
        self.position_x = []
        self.position_y = []
        self.position_all = []
        
        # Boolean variables used for proper display of robot path and global path
        self.bool_goal = False
        self.bool_path = False

        # this determines the iterations the robot tries out a random move to escape local minimi
        self.random_nudge_iters = 8
        self.random_nudge_counter = 0
        self.noise = np.zeros(2)
        self.movement_history = np.zeros((self.random_nudge_iters, 2))
        self.average_movement = 0
        # list containing the last tried random moves and how much average movement they resulted in 
        # the robot will tend its random movements toward those that seem to bring it out of the local minimum more
        self.average_movement_random_motion = np.zeros((self.random_nudge_iters, 3))
        self.cooldown = 10

#------------------------------------------

    def start(self):
        rate = rospy.Rate(self.sample_rate)
        while not rospy.is_shutdown():
            if(self.path_data):
                self.global_path_pub.publish(self.path_data)
           
            self.robot_path_publish()
            net_force = self.compute_attractive_force() ## What should be the net force? self.compute_attractive_force
            net_force += self.compute_repulsive_force()
            #print(f"this is the net force: {net_force}")
            random_force  = self.random_nudge(net_force)
            print(f"net_force: {net_force}")
            print(f"random_force: {random_force}")
            net_force += random_force

            
            self.movement_history[0] = net_force
            self.movement_history = np.roll(self.movement_history, 1, axis=0)
            self.average_movement = np.average(np.linalg.norm(self.movement_history))
            print(f"average movement: {self.average_movement}")

            #self.average_movement_random_motion[0] = np.hstack([self.average_movement, random_force])
            #self.average_movement_random_motion = np.roll(self.average_movement_random_motion, 1, axis=0)
            #print(f"average movement and random force: {self.average_movement_random_motion}")

            self.publish_sum(net_force[0],net_force[1])

            rate.sleep()

#------------------------------------------
    def robot_path_publish(self):
        if(self.odom):
            odom_data = self.odom
            if(self.bool_path == True):
                self.bool_path = False
                self.path_robot = Path()
                self.path_robot.header.frame_id = 'map'
            pose = PoseStamped()
            pose.header = odom_data.header
            pose.pose = odom_data.pose.pose
            self.path_robot.poses.append(pose)
            self.robot_path_pub.publish(self.path_robot)

#------------------------------------------
    def random_nudge(self, net_force):
        """
        Check if robot is not moving, and then if its close or not
        If its not close and is not moving then this will try to nudge the robot
        """
        empty_arr = np.zeros(2)

        if self.odom == None or self.goal == None:
            print("No target")
            return empty_arr 
        
        # if moving and isnt already trying to nudge: return
        if self.average_movement > 0.1 and self.noise[0] == 0:
            print("apparently its moving")
            self.noise = empty_arr
            return empty_arr 

        odom_data = self.odom
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y
        dist_to_goal = np.sqrt((pos_x - self.goal.pose.position.x)**2 + (pos_y - self.goal.pose.position.y)**2)

        # if already close: return
        if dist_to_goal < self.d_star:
            print("already close enough")
            self.noise = empty_arr
            return empty_arr
        
        # if already tried this random motion for self.random_nudge_iters iterations: return same motion
        if self.random_nudge_counter < self.random_nudge_iters:
            print("nudging")
            self.random_nudge_counter += 1
            return np.array([self.noise[0], self.noise[1]])

        # if broken out of local minimum
        if (self.average_movement > 0.8 and self.is_away_from_obstacles()) or self.average_movement > 1.5:
            print("Back to normal")
            if np.linalg.norm(self.noise) > 0.25:
                self.noise = self.noise * 0.8
            else:
                self.noise = np.zeros(2)

            # scrap earlier history, no longer needed
            self.average_movement_random_motion = np.zeros((10, 3))
            return self.noise
        


        away_vector_scaling = 0.7
        away_vector = away_vector_scaling * self.compute_repulsive_force()

        #best_move = np.
        #self.average_movement_random_motion
        self.average_movement_random_motion[0] = np.hstack([self.average_movement, self.noise])
        #best_move_idx = np.argmax(self.average_movement_random_motion[:,0])
        best_move_scaling = np.clip(np.max(self.average_movement_random_motion[:,0]), 0, 1)
        best_move_vector = self.average_movement_random_motion[0,1:]
        best_move = best_move_scaling * best_move_vector
        
        self.noise = np.random.uniform(-0.2+away_vector[0]+best_move[0],0.2+away_vector[0]+best_move[1],2)
        self.random_nudge_counter = 0
        
        self.average_movement_random_motion = np.roll(self.average_movement_random_motion, 1, axis=0)
        return np.array([self.noise[0], self.noise[1]])

#------------------------------------------

    def is_away_from_obstacles(self):
        if self.laser == None or self.odom == None or self.goal == None:
            return True
        
        laser_data = self.laser
        ranges = np.array(laser_data.ranges)
        
        for distance in ranges:
            if (distance < self.q_star):
                return False
        
        return True
                

    def compute_repulsive_force(self):
        if self.laser == None or self.odom == None or self.goal == None:
            return np.array([0.,0.])
    
        laser_data = self.laser
        ranges = np.array(laser_data.ranges)
        angle = laser_data.angle_min
        resolution = laser_data.angle_increment
        vector_sum = np.array([0.0,0.0])
        min_range = 0.1

        # The lidar outputs 360 degree data and therefore we get data of 360 points within the range of the sensor.
        # Each obstacle detected by the sensor will occupy some points out of these 360 points. We treat each single point as 
        # an individual obstacle.
        for distance in ranges:
            x = 0
            y = 0

            if(distance<self.q_star and distance>min_range):
                ## What should be the magnitude of the repulsive force generated by each obstacle point?
                mag = self.eta * (1 / self.q_star - 1 / distance) / (distance**2)
                x = mag * np.cos(angle)
                y = mag * np.sin(angle)

            # This is the negative gradient direction
            vector = np.array([x, y])
            vector_sum += vector ## You need to add the effect of all obstacle points
            angle += resolution

        # Normalization of the repulsive forces
        return np.array([vector_sum[0],vector_sum[1]])*(1/len(ranges))
#------------------------------------------

    def compute_attractive_force(self):
        if self.odom == None or self.goal == None:
            return np.array([0.,0.])

        odom_data = self.odom
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y
        pos = []
        pos.append(pos_x)
        pos.append(pos_y)

        closest_waypoint = []
        while(not closest_waypoint or closest_waypoint is None or not closest_waypoint[1]):
            closest_waypoint = self.closest_waypoint(pos, self.position_all)

        dist_to_goal = np.sqrt((pos_x - self.goal.pose.position.x)**2 + (pos_y - self.goal.pose.position.y)**2)

        index_waypoint = closest_waypoint[0]

        #print(f"dist_to_goal {dist_to_goal}")

        # If the closest waypoint is the last point on the global path then you might want to direct the robot towards the goal position
        ########

        if dist_to_goal <= self.d_star:
            mag = -self.zeta * np.array([pos_x - self.goal.pose.position.x, pos_y - self.goal.pose.position.y])
        else:
            mag = -self.d_star * self.zeta * np.array([pos_x - self.goal.pose.position.x, pos_y - self.goal.pose.position.y]) / dist_to_goal
    
        ## Your stuff

        ########
    
        # You can use a lookahead distance concept in order to drive the robot. Feel free to customize this block of code.
        # The robot follows the waypoint which is say,10 points ahead of the closest waypoint. If the robot is within 10 points
        # from the goal ,use the goal position instead to drive the robot.
        
        ###

        ## Your stuff

        ###

        vector = mag
        #print(vector)

        return np.array([vector[0],vector[1]])
#------------------------------------------

    def closest_waypoint(self,point, points):
        i=0
        pt=[]
        dist = math.inf
        for p in points:
            if(math.dist(p,point)<dist):
                dist = math.dist(p,point)
                pt = p
                i = points.index(pt)
        return [i,pt]
#------------------------------------------

    def handle_laser(self, laser_data):
        self.laser = laser_data
        
#------------------------------------------
    
    def handle_odom(self, odom_data):
        self.odom = odom_data
#------------------------------------------
    
    def handle_goal(self, goal_data):
        self.bool_goal = True
        self.bool_path = True
        self.goal = goal_data
#------------------------------------------
    def publish_sum(self, x, y):
        vector = Point(x, y, 0)
        self.potential_field_pub.publish(vector)

#------------------------------------------
    def publish_dist_to_goal(self, dist):
        dist_to_goal = Float32(dist)
        self.dist_to_goal_pub.publish(dist_to_goal)
#------------------------------------------

    def handle_global_path(self, path_data):
        if(self.bool_goal == True):
            self.bool_goal = False
            self.path_data = path_data
            i=0
            while(i <= len(self.path_data.poses)-1):
                self.position_x.append(self.path_data.poses[i].pose.position.x)
                self.position_y.append(self.path_data.poses[i].pose.position.y)
                i=i+1
            self.position_all = [list(double) for double in zip(self.position_x,self.position_y)]
            self.position_x = []
            self.position_y = []

if __name__ == "__main__":
    pf = potential_field()
    pf.start()