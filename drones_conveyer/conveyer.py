
import time
import numpy as np
from numpy.linalg import norm

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie

from tools import reset_estimator
from drone import Drone


# URI to the Crazyflie to connect to
uri1 = 'radio://0/80/2M/E7E7E7E701'
uri2 = 'radio://0/80/2M/E7E7E7E702'
uri3 = 'radio://0/80/2M/E7E7E7E703'

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    drone1 = Drone(uri1)
    drone2 = Drone(uri2)
    drone3 = Drone(uri3)

    """ First mission """
    with SyncCrazyflie(drone1.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        drone1.scf = scf
        reset_estimator(drone1)
        drone1.start_position_reading() # 20 Hz
        drone1.start_battery_status_reading() # 2 Hz
        time.sleep(1)

        drone1.pose_home = drone1.pose
        print('Home position:', drone1.pose_home)
        print('Battery status %.2f:' %drone1.V_bat)

        print('drone1 is ready for takeoff')
        drone1.takeoff(height=0.9)
        drone1.hover(2)

        """ Flight mission """
        wp_sequence = [
                       [0.4, -0.6, 0.4, 0],
                       [0.4, 0.3, 1.0, 90],
                       [-0.4, 0.3, 1.2, 180],
                       [-0.4, -0.6, 1.4, 270],
                       [0.4, -0.6, 1.2, 360],
                      ]

        for waypoint in wp_sequence:
            drone1.goTo(waypoint)
            drone1.hover(0)

        wp_sequence = [
                       [0.0, 0.0, 1.2, 0],
                       [0.4, 0.3, 1.3, 0],
                       [0.4, -0.6, 1.3, 0],
                       [0.4, -0.6, 0.3, 0],
                       [0.4, 0.3, 0.3, 0],
                      ]

        for waypoint in wp_sequence:
            drone1.goTo(waypoint, vel=0.5)
            drone1.hover(0.5)

        print('Go home before landing...')
        drone1.goTo([drone1.pose_home[0], drone1.pose_home[1], 0.3, 0])
        drone1.hover(0.5)
        drone1.land()
        print('Battery status: %.2f [V]' %drone1.V_bat)


    """ Second mission """
    with SyncCrazyflie(drone2.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        drone2.scf = scf
        reset_estimator(drone2)
        drone2.start_position_reading() # 20 Hz
        drone2.start_battery_status_reading() # 2 Hz
        time.sleep(1)

        drone2.pose_home = drone2.pose
        print('Home position:', drone2.pose_home)
        print('Battery status %.2f:' %drone2.V_bat)

        print('drone2 is ready for takeoff')
        drone2.takeoff(height=0.5)
        drone2.hover(2)

        """ Flight mission """
        wp_sequence = [
                       [0.0, 0.0, 1.3, 0],
                       [0.4, 0.3, 1.3, 0],
                       [-0.4, 0.3, 1.3, 0],
                       [-0.4, -0.6, 1.3, 0],
                       [0.4, -0.6, 1.3, 0],
                      ]

        for waypoint in wp_sequence:
            drone2.goTo(waypoint, vel=0.5)
            drone2.hover(1)

        print('Go home before landing...')
        drone2.goTo([drone2.pose_home[0], drone2.pose_home[1], 0.3, 0])
        drone2.hover(2)
        drone2.land()
        print('Battery status: %.2f [V]' %drone2.V_bat)


    """ Fird mission """
    with SyncCrazyflie(drone3.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        drone3.scf = scf
        reset_estimator(drone3)
        drone3.start_position_reading()
        drone3.start_battery_status_reading() # 2 Hz
        time.sleep(1)

        drone3.pose_home = drone3.pose
        print('Home position:', drone3.pose_home)
        print('Battery status %.2f:' %drone3.V_bat)

        drone3.takeoff(height=0.3)
        drone3.hover(1)

        drone3.goTo([0.0, -0.3, 1.1, 0], vel=0.5)
        drone3.hover(2)

        drone3.trajectory()

        print('Go home before landing...')
        drone3.goTo([drone3.pose_home[0], drone3.pose_home[1], 0.3, 0])
        drone3.hover(2)
        drone3.land()
        print('Battery status: %.2f [V]' %drone3.V_bat)

