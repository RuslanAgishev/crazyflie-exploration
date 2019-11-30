#!/usr/bin/env python

# -*- coding: utf-8 -*-

"""
This example is intended to work with the Loco Positioning System in TWR TOA
mode. It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints.
"""
import time
import numpy as np
from numpy.linalg import norm

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from drone_multiranger import DroneMultiranger
from threading import Thread


import sys, termios, tty, os, time
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def msg_def_PoseStamped(pose, orient):
    worldFrame = "base_link"
    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.position.z = pose[2]
    msg.pose.orientation.x = orient[0]
    msg.pose.orientation.y = orient[1]
    msg.pose.orientation.z = orient[2]
    msg.pose.orientation.w = orient[3]
    msg.header.seq += 1
    return msg
def publish_path(path, pose, orient=[0,0,0,1], topic_name='test_path', limit=-1):
    msg = msg_def_PoseStamped(pose, orient)
    path.header = msg.header
    path.poses.append(msg)
    if limit>0:
        path.poses = path.poses[-limit:]
    pub = rospy.Publisher(topic_name, Path, queue_size=1)
    pub.publish(path)
def publish_pose(pose, orient=[0,0,0,1], topic_name='test_path'):
    msg = msg_def_PoseStamped(pose, orient)
    pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
    pub.publish(msg)

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            print("{} {} {}".
                  format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(scf)


def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(scf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', controller)


def prepare(drone):
    activate_high_level_commander(drone.scf)
    reset_estimator(drone.scf)
    activate_mellinger_controller(drone.scf, False)
    drone.estimated_pose = 1
    # print("Drone is estimated its position: ", drone.id)


def fly(drone):
    drone.cf.commander.send_position_setpoint(drone.sp[0], drone.sp[1], drone.sp[2], drone.sp[3])

def takeoff(drone, height=0.3):
    # takeoff to z=0.3 m:
    print('Takeoff...')
    drone.sp = np.zeros(4); drone.sp[:3] = drone.position
    dz = 0.02
    for i in range(int(height/dz)):
        drone.sp[2] += dz
        if TO_FLY: fly(drone)
        time.sleep(0.1)

def land(drone):
    print('Landing...')
    while drone.sp[2]>-0.1:
        drone.sp[2] -= 0.02
        if TO_FLY: fly(drone)
        time.sleep(0.1)
    stop(drone)
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)
def stop(drone):
    drone.cf.commander.send_stop_setpoint()

def goTo(drone, goal, pos_tol=0.03, yaw_tol=3):
    goal = np.array(goal)
    print('Going to', goal)
    if drone.sp is None:
        drone.sp = np.zeros(4); drone.sp[:3] = drone.pose
    while norm(goal[:3] - drone.sp[:3]) > pos_tol or norm(drone.sp[3]-goal[3]) > yaw_tol:
        n = normalize(goal[:3] - drone.sp[:3])
        drone.sp[:3] += 0.03 * n # position setpoints
        drone.sp[3] += 3 * np.sign( goal[3] - drone.sp[3] ) # yaw angle
        # print('Yaw', drone.sp[3], 'yaw diff', norm(drone.sp[3]-goal[3]))
        if TO_FLY: fly(drone)
        time.sleep(0.1)
def normalize(vector):
    vector = np.array(vector)
    v_norm = vector / norm(vector) if norm(vector)!=0 else np.zeros_like(vector)
    return v_norm

def hover(drone, t_hover=2):
    t0 = time.time()
    print('Holding position for %.2f [sec]...' %t_hover)
    while time.time() - t0 < t_hover:
        if TO_FLY: fly(drone)
        time.sleep(0.1)


def spiral_trajectory(drone, initial_angle=0, t_wait=0):
    '''
    The group of 3 drones is initialy placed as follows:
                  1^         3^
 
                        0

                        2^
    where 0 denotes origin, and numbers 1,2,3 labels of the
    crazyflies, ^ shows orientation along X-axis.
    The swarm ascends via spiral to the maximum height=1.6 [m].
    The drones start flying one by one with a delay between neighbouring
    UAVs equal to 5.33 [sec]. Ones the maximum height is reached,
    each quadrotor performes a circular flight at a constant height
    and then starts to descend along spiral trajectory.
    One circular revolution consumes 8 [sec] of time.
    Total time of trajectory execution is 8x3 + 8 + 8x3 = 8x7 = 56 [sec].
    '''
    drone.path = Path()
    label = drone.id
    time.sleep(t_wait)

    # print('Ready to fly', drone.id)
    commander = drone.cf.commander
    angular_range = np.linspace(0+initial_angle, 2*np.pi+initial_angle, 80)
    R = 0.7; h = 0.2; dh = 0.005
    numiters = 3
    
    takeoff(drone)

    # Ascedning via spiral
    for _ in range(numiters):
        # one circle in spiral trajectory
        for t in angular_range:
            sp = [R*np.cos(t), R*np.sin(t), h, 0]
            if TO_FLY:
                commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
            publish_path(drone.path, sp[:3], topic_name='path'+label)
            publish_pose(sp[:3], topic_name='pose'+label)
            h += dh
            time.sleep(0.1)

    # 1 circle on maximum height
    for t in angular_range:
        sp = [R*np.cos(t), R*np.sin(t), h, 0]
        if TO_FLY:
            commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
        publish_path(drone.path, sp[:3], topic_name='path'+label)
        publish_pose(sp[:3], topic_name='pose'+label)
        time.sleep(0.1)

    # Decending via spiral
    for _ in range(numiters):
        # one circle in spiral trajectory
        for t in angular_range:
            sp = [R*np.cos(t), R*np.sin(t), h, 0]
            if TO_FLY:
                commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
            publish_path(drone.path, sp[:3], topic_name='path'+label)
            publish_pose(sp[:3], topic_name='pose'+label)
            h -= dh
            time.sleep(0.1)

    goTo(drone, [drone.pose_home[0], drone.pose_home[1], h, 0])
    hover(drone, t_hover=1.2)
    land(drone)

TO_FLY = 1

# URI to the Crazyflie to connect to
URI1 = 'radio://0/80/2M/E7E7E7E701'
URI2 = 'radio://0/80/2M/E7E7E7E702'
URI3 = 'radio://0/80/2M/E7E7E7E703'

if __name__ == '__main__':
    rospy.init_node('drone_multiranger')

    drone1 = DroneMultiranger(URI1)
    drone2 = DroneMultiranger(URI2)
    drone3 = DroneMultiranger(URI3)
    time.sleep(4)
    
    drone1.pose_home = drone1.position
    drone2.pose_home = drone2.position
    drone3.pose_home = drone3.position

    print('Home positions:', drone1.pose_home, drone2.pose_home, drone3.pose_home)

    if TO_FLY:
        th1 = Thread(target=prepare, args=(drone1,) )
        th2 = Thread(target=prepare, args=(drone2,) )
        th3 = Thread(target=prepare, args=(drone3,) )
        th1.start(); th2.start(); th3.start()
        th1.join(); th2.join(); th3.join()
        print('Number of drones estimated pose: ',
              drone1.estimated_pose + drone2.estimated_pose + drone3.estimated_pose)

    print("Press F to fly...")
    while True:
        char = getch()
        if (char == "f"):
            break

    print("Starting the mission!")
    th1 = Thread(target=spiral_trajectory, args=(drone1, np.pi/3, 0) )
    th2 = Thread(target=spiral_trajectory, args=(drone2, np.pi, 5.33) )
    th3 = Thread(target=spiral_trajectory, args=(drone3, 5*np.pi/3, 10.66) )
    th1.start(); th2.start(); th3.start()
    th1.join(); th2.join(); th3.join()


