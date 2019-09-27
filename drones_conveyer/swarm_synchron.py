# -*- coding: utf-8 -*-

"""
Simple example of a swarm using the High level commander.

The swarm takes off and flies a synchronous square shape before landing.
The trajectories are relative to the starting positions and the Crazyfles can
be at any position on the floor when the script is started.

This example is intended to work with any absolute positioning system.
It aims at documenting how to use the High Level Commander together with
the Swarm class.
"""
import time
import numpy as np
import threading

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger



V_BATTERY_TO_GO_HOME = 3.4
V_BATTERY_CHARGED = 3.9

class Drone:
    def __init__(self, scf):
        self.scf = scf
        self.pose = None
        self.pose_home = None
        self.start_position_reading()
        self.start_battery_status_reading()

    def position_callback(self, timestamp, data, logconf):
        x = data['kalman.stateX']
        y = data['kalman.stateY']
        z = data['kalman.stateZ']
        self.pose = np.array([x, y, z])
    def start_position_reading(self):
        log_conf = LogConfig(name='Position', period_in_ms=50) # read position with 20 Hz rate
        log_conf.add_variable('kalman.stateX', 'float')
        log_conf.add_variable('kalman.stateY', 'float')
        log_conf.add_variable('kalman.stateZ', 'float')

        self.scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.position_callback)
        log_conf.start()

    def battery_callback(self, timestamp, data, logconf):
        self.V_bat = data['pm.vbat']
        print('Battery status: %.2f [V]' %self.V_bat)
        if self.V_bat <= V_BATTERY_TO_GO_HOME:
            self.battery_state = 'needs_charging'
            print('Battery is not charged: %.2f' %self.V_bat)
        elif self.V_bat >= V_BATTERY_CHARGED:
            self.battery_state = 'fully_charged'
    def start_battery_status_reading(self):
        log_conf = LogConfig(name='Battery', period_in_ms=500) # read battery status with 2 Hz rate
        log_conf.add_variable('pm.vbat', 'float')
        self.scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.battery_callback)
        log_conf.start()


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

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

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


def run_shared_sequence(scf, drone):
    activate_mellinger_controller(scf, False)

    flight_time = 3.5

    commander = scf.cf.high_level_commander

    commander.takeoff(0.3, 2.0)
    time.sleep(3)

    for goal in drone.waypoints:
        print('Going to', goal)
        commander.go_to(goal[0], goal[1], goal[2], goal[3]/180*3.14, flight_time, relative=False)
        time.sleep(flight_time)

    commander.land(0.0, 1.0)
    time.sleep(1)
    commander.stop()

def run_shared_sequence_battery_check(scf, drone):
    activate_mellinger_controller(scf, False)

    flight_time = 3.5

    commander = scf.cf.high_level_commander
    if drone.battery_state == 'fully_charged':
        commander.takeoff(0.3, 2.0)
        time.sleep(3)

        for goal in drone.waypoints:
            if not drone.battery_state == 'needs_charging':
                print('Going to', goal)
                commander.go_to(goal[0], goal[1], goal[2], goal[3]/180*3.14, flight_time, relative=False)
                time.sleep(1)

        commander.go_to(drone.waypoints[-1][0], 
                        drone.waypoints[-1][1],
                        drone.waypoints[-1][2],
                        0, flight_time, relative=False)
        time.sleep(1)

        commander.land(0.0, 1.0)
        time.sleep(1)
        commander.stop()


r = 0.5
# x[m], y[m], z[m], yaw[deg]
waypoints1 = [
            (0.0, r, 1.2, 0),
            (r, 0.0, 1.3, -90),
            (0.0, -r, 1.5, -180),
            (-r, 0.0, 1.2, -270),
            (0.0, r, 1.2, -360),
            (r, 0.0, 1.3, -270),
            (0.0, -r, 1.5, -180),
            (-r, 0.0, 1.2, -90),
            ]
# x[m], y[m], z[m], yaw[deg]
waypoints2 = [
            (0.0, 0.0, 1.5, 0), 
            (0.0, 0.0, 1.3, -90),
            (0.0, 0.0, 1.3, -180 ),
            (0.0, 0.0, 1.2, -90),
            (0.0, 0.0, 1.6, 0 ),
            (0.0, 0.0, 1.6, -90),
            (0.0, 0.0, 1.6, -180 ),
            (0.0, 0.0, 1.6, -90),
            ]
# x[m], y[m], z[m], yaw[deg]
waypoints3 = [
            (0.0, -r, 1.2, 0),
            (-r, 0.0, 1.3, -90),
            (0.0, r, 1.5, -180),
            (r, 0.0, 1.2, -270),
            (0.0, -r, 1.2, -360),
            (-r, 0.0, 1.3, -270),
            (0.0, r, 1.5, -180),
            (r, 0.0, 1.2, -90),
            ]
waypoints = [
    waypoints1,
    waypoints2,
    waypoints3,
]

URI1 = 'radio://0/80/2M/E7E7E7E701'
URI2 = 'radio://0/80/2M/E7E7E7E702'
URI3 = 'radio://0/80/2M/E7E7E7E703'
uris = {
    URI1,
    URI2,
    # URI3,
}



if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(activate_high_level_commander)
        swarm.parallel_safe(reset_estimator)

        drones = [None for _ in range(len(uris))]
        i = 0
        for uri, scf in swarm._cfs.items():
            drones[i] = Drone(scf)
            time.sleep(1.0)
            drones[i].pose_home = drones[i].pose
            waypoints[i].append( (drones[i].pose_home[0],
                                  drones[i].pose_home[1],
                                  drones[i].pose_home[2]+0.1, 0) )
            drones[i].waypoints = waypoints[i]
            i+=1

        wp_args = {
            URI1: [drones[0]],
            URI2: [drones[1]],
            # URI3: [drones[2]],
        }

        swarm.parallel_safe(run_shared_sequence, args_dict=wp_args)
        # swarm.parallel_safe(run_shared_sequence_battery_check, args_dict=wp_args)

