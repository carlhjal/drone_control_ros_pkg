#!/usr/bin/env python3  
import rospy
import tf2_ros
import gazebo_msgs.msg
import geometry_msgs.msg
import time
from  get_pointcloud.msg import  Obsposelist
import pdb
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf_conversions
from #TODO include odometry from nav_msgs.msg 

broadcaster = None
last_published = None
publish_frequency = None
odom_publisher = None

def get_odom_base_footprint(transform,data):
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_footprint"
    transform.transform.translation.x =data.pose.pose.position.x  #TODO get x position from the data
    transform.transform.translation.y = #TODO get y position from the data
    transform.transform.translation.z = 0
    #TODO Question why are we setting transform.transform.translation.z to 0
    
    #TODO get the drone's quaternion from the data.pose 
    #TODO covert the drone's quaternion to  the roll,pitch , yaw of drone 
    #above two task can be done in one line
    (drone_roll, drone_pitch, drone_yaw) = #TODO  get drone quaternion from data and then use euler_from_quaternion function  to covert quat to euler
    
    #TODO set the roll and pitch to zero and convert the euler angles to quaternion 
    base_foot_print_quaternion = quaternion_from_euler( #TODO )
    
    #TODO Question why we are setting roll and pitch value to zero and not yaw?
    
    transform.transform.rotation.x = base_foot_print_quaternion[0]
    transform.transform.rotation.y = #TODO
    transform.transform.rotation.z = #TODO
    transform.transform.rotation.w = #TODO
   
    return transform
    



def pub_callback(data):
    global last_published, broadcaster, publish_frequency, odom_publisher
    transforms =[]
    
    #get the bebop_orientation from data
    bebop_orientation = #ODO                                      
    
    
    #Following lines create a Odometery messages and publish it in odom_publisher
    bebop_odom = Odometry()
    bebop_odom.header.frame_id = 'odom'
    bebop_odom.child_frame_id = 'bebop_odom'
    bebop_odom.pose.pose.position = data.pose.position
    bebop_odom.pose.pose.orientation = bebop_orientation
    odom_publisher.publish(bebop_odom)
    
    
    #following three lines create a new tf frame connected odom to base_footprint
    transform = geometry_msgs.msg.TransformStamped()
    transform = get_odom_base_footprint(transform,bebop_odom)
    broadcaster.sendTransform(transform)


if __name__ == '__main__':
    rospy.init_node('drone_optitrack_pose_broadcaster')
    odom_publisher = rospy.Publisher("bebop_optitrack/odom", Odometry, queue_size = 1)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    publish_frequency = rospy.get_param("publish_frequency", 10)
    #TODO create a subsrciber which subscribe to /vrpn_client_node/Bebop/pose of type PoseStamped and use pub_callback function as callback
    #one line

    rospy.spin()