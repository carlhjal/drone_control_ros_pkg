#!/usr/bin/env python3

import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from gazebo_msgs.msg import ModelStates
import numpy as np
import tf.transformations
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2
from message_filters import ApproximateTimeSynchronizer

class PointcloudFromGz:
    def __init__(self):
        rospy.init_node("obs_pointcloud_generator")
        self.obstacles = np.array([])
        self.num_points = 50
        self.pointcloud_pub = rospy.Publisher("Obstacle_pointcloud", PointCloud2, queue_size=10)
        self.gz_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.publish_pc)
        rospy.spin()

    def obstacle_callback(self, msg: ModelStates):
        if len(msg.name) <= 2:
            return
        
        obstacles = []
        for i in range(2, len(msg.name)):
            if msg.name[i].startswith("unit"):
                pose = msg.pose[i]
                x = pose.position.x
                y = pose.position.y
                obstacles.append([x, y])

        self.obstacles = np.array(obstacles)

    def create_a_circle(self, center_x: np.ndarray, center_y: np.ndarray, radius=1, height=1):
        list_circular_point= []
        points = np.arange(0,2*np.pi,np.pi/180)
        x_points = center_x.reshape(-1,1) + radius * np.cos(points) #TODO #get list of  x coordinates which is on the circumference of circle with center center_x 
        y_points = center_y.reshape(-1,1) + radius * np.sin(points) #get list of  y coordinates which is on the circumference of circle with center center_y 
        z_points = height * np.ones((center_x.shape[0],points.shape[0])) #get the list z_points , in our case z points represent constant height h ,defined above .This list should be of same lenght of x_points  

        return x_points, y_points, z_points
    
    def publish_pc(self, msg):
        """Generate points for obstacles as a point cloud."""

        self.obstacle_callback(msg)

        if self.obstacles.size == 0:
            return
        
        center_x = self.obstacles[:,0]
        center_y = self.obstacles[:,1]

        xp, yp, zp = self.create_a_circle(center_x, center_y)
        xp,yp,zp = xp.reshape(-1),yp.reshape(-1),zp.reshape(-1)
        cloud_points = np.vstack((xp,yp,zp)).T

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'odom'
        #create pcl from points
        scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
        self.pointcloud_pub.publish(scaled_polygon_pcl)


if __name__ == "__main__":
    node = PointcloudFromGz()