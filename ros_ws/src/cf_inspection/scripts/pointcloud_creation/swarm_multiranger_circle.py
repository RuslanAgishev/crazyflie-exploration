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
from multiranger import DroneMultiranger
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

def prepare(scf):
    activate_high_level_commander(scf)
    reset_estimator(scf)
    activate_mellinger_controller(scf, False)

def fly(drone):
    sp = drone.sp
    drone.cf.commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
def land(drone):
    print('Landing...')
    while drone.sp[2]>-0.1:
        drone.sp[2] -= 0.02
        fly(drone)
        time.sleep(0.1)
    stop(drone)
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)
def stop(drone):
    drone.cf.commander.send_stop_setpoint()

def circle_trajectory(drone, initial_angle=0):
    '''
    The group of 3 drones is initialy placed as follows:
                  1^         3^
 
                        0

                        2^
    where 0 denotes origin, and numbers 1,2,3 labels of the
    crazyflies, ^ shows orientation along X-axis.
    The swarm takes off at initial height, h0=0.2 [m], and then
    the drones performe simultaneously circular trajectory at a constant height.
    After the full circle is executed, the heigt is increased with value dh=0.4 [m].
    Then the swarm again performce a revolution at the new constant height.
    Ones the maximum height, 1.6 [m] is reached, the drones start descending
    in the same manner.
    One circular revolution consumes 8 [sec] of time.
    Total time of trajectory execution is 8x3 + 8 + 8x3 = 8x7 = 56 [sec].
    '''
    def fly_one_circle():
        # circular trajectory
        for t in angular_range:
            # Check battery status
            try:
                if drone.battery_state == 'needs_charging':
                    print('Going home to CHARGE the battery')
                    break
            except:
                pass
            yaw = (t - np.pi/2) % (2*np.pi)
            sp = [R*np.cos(t), R*np.sin(t), h, 180*yaw/np.pi]
            drone.sp = sp
            if TO_FLY: commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
            q = quaternion_from_euler(0,0,yaw)
            publish_pose(sp[:3], orient=q, topic_name='pose'+label)
            publish_path(drone.path, sp[:3], topic_name='path'+label)
            time.sleep(dt)


    commander = drone.cf.commander
    hl_commander = drone.cf.high_level_commander
    label = drone.processing.id
    drone.path = Path()

    N_samples = 160; dt = 0.1
    angular_range = np.linspace(0+initial_angle, 2*np.pi+initial_angle, N_samples)
    R = 0.7; h = 0.1; dh = 0.1
    numiters = 5
    t0 = time.time()

    if TO_FLY:
        hl_commander.takeoff(h, 1.0)
        time.sleep(1)
        # Going to initial locations
        flight_time = 3.0
        hl_commander.go_to(R*np.cos(initial_angle),
                           R*np.sin(initial_angle),
                           h,
                           initial_angle - np.pi/2,
                           flight_time, relative=False)
        time.sleep(flight_time)
        # hl_commander.land(0.0, 2.0)
        # time.sleep(2)
        # hl_commander.stop()
        # time.sleep(0.1)

    print('Drone velocity: %.2f [m/s]' %(2*np.pi*R/(N_samples*0.1)))
    drone.sp = np.array([R*np.cos(initial_angle),
                         R*np.sin(initial_angle),
                         h,
                         initial_angle - np.pi/2])

    # Trajectory
    for _ in range(numiters):
        print('Height: %.2f [m]' %h)
        fly_one_circle()
        h += dh

    h -= dh /2.
    print('Height: %.2f [m]' %h)
    fly_one_circle()

    for _ in range(numiters):
        h -= dh
        print('Height: %.2f [m]' %h)
        fly_one_circle()
    print('Time passed: %2.f [sec]' %(time.time()-t0))
    
    land(drone)
    hl_commander.stop()
    time.sleep(0.1)
    commander.send_stop_setpoint()
    time.sleep(0.1)

TO_FLY = 1

# URI to the Crazyflie to connect to
URI1 = 'radio://0/80/2M/E7E7E7E702'
URI2 = 'radio://0/80/2M/E7E7E7E703'
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

    # print('Home positions:', drone1.pose_home, drone2.pose_home, drone3.pose_home)

    if TO_FLY:
        th1 = Thread(target=prepare, args=(drone1.scf,) )
        th2 = Thread(target=prepare, args=(drone2.scf,) )
        th3 = Thread(target=prepare, args=(drone3.scf,) )
        th1.start(); th2.start(); th3.start()
        th1.join(); th2.join(); th3.join()

    print("Press F to fly...")
    while True:
        char = getch()
        if (char == "f"):
            break

    th1 = Thread(target=circle_trajectory, args=(drone1, np.pi/2,) )
    th2 = Thread(target=circle_trajectory, args=(drone2, 3*np.pi/2,) )
    th3 = Thread(target=circle_trajectory, args=(drone3, 5*np.pi/3,) )
    th1.start(); th2.start(); th3.start()
    th1.join(); th2.join(); th3.join()

