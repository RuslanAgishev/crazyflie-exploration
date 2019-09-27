import math
import matplotlib.pyplot as plt
import numpy as np
from grid_based_sweep_coverage_path_planner import planning
from connect_crazyflie import reset_estimator
from connect_crazyflie import activate_high_level_commander
from connect_crazyflie import activate_mellinger_controller

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import time

import rospy
from multiranger import DroneMultiranger

def waypts2setpts(P, drone_vel=0.5, update_rate=10):
    """
    construct a long array of setpoints, traj_global, with equal inter-distances, dx,
    from a set of via-waypoints, P = [[x0,y0], [x1,y1], ..., [xn,yn]]
    """
    V = drone_vel # [m/s]
    freq = update_rate; dt = 1./freq
    dx = V * dt
    traj_global = np.array(P[0])
    for i in range(1,len(P)):
        A = P[i-1]
        B = P[i]

        n = (B-A) / np.linalg.norm(B-A)
        delta = n * dx
        N = int( np.linalg.norm(B-A) / np.linalg.norm(delta) )
        sp = A
        traj_global = np.vstack([traj_global, sp])
        for i in range(N):
            sp += delta
            traj_global = np.vstack([traj_global, sp])
        sp = B
        traj_global = np.vstack([traj_global, sp])

    return traj_global

def planning_animation(ox, oy, reso, params):  # pragma: no cover
    px, py = planning(ox, oy, reso)

    P = np.hstack([ np.array(px)[:,np.newaxis], np.array(py)[:,np.newaxis] ])
    traj_global = waypts2setpts(P)

    # animation
    if params.do_animation:
        for ipx, ipy in zip(traj_global[:,0], traj_global[:,1]):
            plt.cla()
            plt.plot(ox, oy, "-xb")
            plt.plot(px, py, "-r")
            plt.plot(ipx, ipy, "or")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.1)

    plt.cla()
    plt.plot(ox, oy, "-xb")
    plt.plot(px, py, "-r")
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.1)

    return traj_global
    
def takeoff(drone, height=0.2):
    # takeoff to z=0.3 m:
    print('Takeoff...')
    drone.sp = np.zeros(4); drone.sp[:3] = drone.position
    dz = 0.02
    for i in range(int(height/dz)):
        drone.sp[2] += dz
        fly(drone)
        time.sleep(0.1)
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
def goTo(drone, goal, pos_tol=0.03, yaw_tol=3):
    def normalize(vector):
        vector = np.array(vector)
        v_norm = vector / np.linalg.norm(vector) if np.linalg.norm(vector)!=0 else np.zeros_like(vector)
        return v_norm
    goal = np.array(goal)
    print('Going to', goal)
    if drone.sp is None:
        drone.sp = np.zeros(4); drone.sp[:3] = drone.pose
    while np.linalg.norm(goal[:3] - drone.sp[:3]) > pos_tol or np.linalg.norm(drone.sp[3]-goal[3]) > yaw_tol:
        n = normalize(goal[:3] - drone.sp[:3])
        drone.sp[:3] += 0.03 * n # position setpoints
        drone.sp[3] += 3 * np.sign( goal[3] - drone.sp[3] ) # yaw angle
        # print('Yaw', drone.sp[3], 'yaw diff', np.linalg.norm(drone.sp[3]-goal[3]))
        fly(drone)
        time.sleep(0.1)
def hover(drone, t_hover=2):
    t0 = time.time()
    print('Holding position for %.2f [sec]...' %t_hover)
    while time.time() - t0 < t_hover:
        fly(drone)
        time.sleep(0.1)

def run_sequence(drone, sequence, params):
    commander = drone.scf.cf.commander

    if params.toFly:
        takeoff(drone)
        # Going to initial locations
        goTo(drone, [sequence[0,0], sequence[0,1], sequence[0,2], sequence[0,3]])

    for sp in sequence:
        if params.toFly: commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
        time.sleep(0.2)

    # Landing...
    while sp[2] > -0.1:
        sp[2] -= 0.02
        if params.toFly: commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
        time.sleep(0.1)

    if params.toFly: commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

def prepare(drone):
    scf = drone.scf
    activate_high_level_commander(scf)
    reset_estimator(scf)
    activate_mellinger_controller(scf, False)

class Params:
    def __init__(self):
        self.numiters = 200
        self.vel = 0.5 # [m/s]
        self.uri = 'radio://0/80/2M/E7E7E7E703'
        self.flight_height = 0.5 # [m]
        self.toFly = 0
        self.check_battery = 1
        self.do_animation = 0

if __name__ == '__main__':
    rospy.init_node('random_walk')
    params = Params()
    drone = DroneMultiranger(params.uri)
    time.sleep(3)
    drone.pose_home = drone.position
    print('Home positions:', drone.pose_home)

    SCALE = 1.0
    ox = np.array([-1.0, 1.0,  1.0, -1.0, -1.0]) * SCALE
    oy = np.array([1.0,  1.0, -1.0, -1.0,  1.0]) * SCALE
    reso = 0.2

    traj_global = planning_animation(ox, oy, reso, params)

    px, py = traj_global[:,0], traj_global[:,1]
    px = px[np.newaxis].T; py = py[np.newaxis].T # making column vectors to stuck in sequence
    sequence = np.hstack([px, py, params.flight_height*np.ones_like(px), np.zeros_like(px)])

    if params.toFly:
        prepare(drone)
        raw_input('Press Enter to fly...')

        plt.close('all')

        print("Flight!!")
        run_sequence(drone, sequence, params)

    plt.show()