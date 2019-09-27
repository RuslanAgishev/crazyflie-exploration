#!/usr/bin/env python

# -*- coding: utf-8 -*-

"""
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and send a sequence of setpoints,
one every 5 seconds.

This example is intended to work with the Loco Positioning System in TWR TOA
mode. It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints.
"""

import time
import numpy as np
from numpy.linalg import norm
from threading import Thread

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
from cflib.positioning.motion_commander import MotionCommander

import rospy
from multiranger1 import DroneMultiranger


# URI to the Crazyflie to connect to
URI1 = 'radio://0/80/2M/E7E7E7E703'


if __name__ == '__main__':
    rospy.init_node('mission_pointcloud')
    cflib.crtp.init_drivers(enable_debug_driver=False)

    drone = DroneMultiranger(URI1)

    drone.mc = MotionCommander(drone.cf)
    drone.mc.take_off(0.1, 0.1)
    time.sleep(1)

    for _ in range(5):
        drone.mc.circle_right(0.9, velocity=0.5, angle_degrees=360)
        drone.mc.up(0.1)
        time.sleep(1)

    drone.mc.down(0.05)
    time.sleep(1)

    for _ in range(5):
        drone.mc.circle_right(0.9, velocity=0.5, angle_degrees=360)
        drone.mc.down(0.1)
        time.sleep(1)

    drone.mc.land(0.2)
    time.sleep(1)
