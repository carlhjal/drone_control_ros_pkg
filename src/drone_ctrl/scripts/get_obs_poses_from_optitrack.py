#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import time
from  #TODO import OBSposelist

def pub_callback(data):
    global last_published
    if last_published and publish_frequency > 0.0 and time.time() - last_published <= 1.0 / publish_frequency:
        return
    transforms = []
    obstacles_list = Obsposelist() # TODO define a new message object for Obsposelist  type
    for i in range(len(data.name)):
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"
        transform.child_frame_id = "obstacle_1"
        transform.transform.translation.x = data.pose[i].position.x
        transform.transform.translation.y =  # TODO
        transform.transform.translation.z =  # TODO
        transform.transform.rotation.w =  # TODO
        transform.transform.rotation.x =  # TODO
        transform.transform.rotation.y =  # TODO
        transform.transform.rotation.z =  # TODO
        transforms.append(transform)
        obstacles_list.obs_poses_list.append(transform)
        # TODO
    broadcaster.sendTransform(transforms)


    obs_list_pub.publish(obstacles_list)

    last_published = time.time()

if __name__ == '__main__':
    rospy.init_node('obs_optitrack_pose_broadcaster')
    rospy.Rate(10)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    publish_frequency = rospy.get_param("publish_frequency", 10)

    obs_list_pub = rospy.Publisher("/obs_poses_list", Obsposelist, queue_size=10)
    
    
    last_published = None
    #TODO create a subsrciber which subscribe to /vrpn_client_node/obstacle_1/pose of type PoseStamped and use pub_callback function as callback
    #one line
    rospy.spin()