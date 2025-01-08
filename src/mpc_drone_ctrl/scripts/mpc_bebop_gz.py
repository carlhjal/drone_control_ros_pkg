#!/usr/bin/env python3


import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from gazebo_msgs.msg import ModelStates
import numpy as np
import jax.numpy as jnp
from npy_fn import Mppi_npy
from mppi_fn import MPPIControllerForPathTracking
import tf.transformations
from time import time
from jax import random, lax
import threading

husky_state = [0., 0., 0.]
obstacles = []
msg = geometry_msgs.msg.Twist()

def publish_command(pub, optimal_input):
    msg.linear.x = optimal_input[0]
    msg.angular.z = optimal_input[1]
    pub.publish(msg)

def odom_callback(msg):
    global husky_state
    pos = msg.pose.pose
    quat = pos.orientation

    angles = tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
    th = angles[2]
    pose = [pos.position.x, pos.position.y, th]
    husky_state = pose

def obstacle_callback(msg):
    global obstacles
    if len(msg.name) <= 2:
        return
    
    obstacles = []
    for i in range(2, len(msg.name)):
        pose = msg.pose[i]
        x = pose.position.x
        y = pose.position.y
        r = 0.3
        obstacles.append([x, y, r])
        #obstacles.append([msg.pose[i].position.x, msg.pose[i].position.y])

def run_node():
    rospy.init_node('bebop_control',anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, obstacle_callback)
    rospy.Subscriber("/bebop/odom", nav_msgs.msg.Odometry, odom_callback)
    #rospy.sleep(0.02)

def optimizer():
    delta_t = 0.05
    sim_steps = 10000

    #ref_path = np.genfromtxt('ovalpath.csv', delimiter=',', skip_header=1)
    r = 4
    points = 1000
    pi = np.linspace(0, 2*np.pi, points)
    x = r*np.cos(pi)
    y = r*np.sin(pi)
    yaw = np.zeros(points)
    ref_v = np.ones(points)*0.5
    ref_path = np.vstack((x,y,yaw,ref_v)).T
    
    max_omega = 0.785
    max_vel = 2.0
    T = 50

    pub = rospy.Publisher('bebop/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)
    rate = rospy.Rate(1/delta_t)
    
    init_state = jnp.array([0.0,0.0,0.0])
    current_state = jnp.copy(init_state)
    mppi = MPPIControllerForPathTracking(
        delta_t = delta_t*2.0, # [s]
        max_omega = max_omega, # [rad]      
        max_vel = max_vel, # [m/s^2]
        horizon_step_T = T, # [steps]
        number_of_samples_K = 100, #500 [samples]
        param_exploration = 0.0,
        param_lambda = 100.0,
        param_alpha = 0.98,
        sigma = jnp.diag(jnp.array([0.0075,0.5])),
        stage_cost_weight = jnp.array([50.0, 50.0, 50.0, 1.0]), # weight for [x, y, yaw, v]
        terminal_cost_weight = jnp.array([50.0, 50.0, 50.0, 1.0]), # weight for [x, y, yaw, v]
        window_size= T,
        obstacle_circles = jnp.array(obstacles)
    )
    npy_ob = Mppi_npy(np.array(ref_path),T)

    u_prev = jnp.zeros((T, 2))

    key = random.PRNGKey(0)
    # simulation loop
    for i in range(sim_steps):
        key, subkey = random.split(key)
        start_time = time()
        current_state = jnp.array(husky_state)
        ref_x, ref_y, ref_yaw, ref_v = npy_ob.get_nearest_waypoint(np.asarray(current_state[0]),np.asarray(current_state[1]))
        
        mppi.obstacle_circles = jnp.array(obstacles)

        ref_x = jnp.asarray(ref_x)
        ref_y = jnp.asarray(ref_y)
        ref_yaw = jnp.asarray(ref_yaw)
        ref_v = jnp.asarray(ref_v)
        # calculate input force with MPPI
        optimal_input, optimal_input_sequence, optimal_traj, sampled_traj_list, u_prev = mppi.calc_control_input(current_state, ref_x, ref_y,\
                                                                                                                    ref_yaw, ref_v, key, u_prev)
        print("elapsed=",time()-start_time)
        # print(optimal_input)
        # print(mppi.u_prev)
        # exit(0)
        # print current state and input force
        print(f"Time: {i*delta_t:>2.2f}[s], x={current_state[0]:>+3.3f}[m], y={current_state[1]:>+3.3f}[m], yaw={current_state[2]:>+3.3f}[rad], omega={optimal_input[0]:>+6.2f}[rad/s], vel={optimal_input[1]:>+6.2f}[m/s]")

        optimal_traj_new = optimal_traj[:, 0:2]
        sampled_traj_list_new = sampled_traj_list[:, :, 0:2]

        current_state = mppi.rk4(current_state,optimal_input)
        print(optimal_input)

        publish_command(pub, optimal_input)
        rate.sleep()

if __name__ == "__main__":
    run_node()
    optimizer()

    #while True:
    #    print(f"husky state: {husky_state}")
    #    print(f"obstacles: {obstacles}")


