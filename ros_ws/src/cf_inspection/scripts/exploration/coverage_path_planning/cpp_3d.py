"""
Coverage Path Planning algorithm implementation for a drone (tested on Crazyflie 2.1)
equipped with 4 ranger sensors (front, back, left and right)
for obstacles detection. Flight volume is defined as 
a polygon base prism in 3D. The prism and its location in space
is defined by the shape of the vertices of the polygon in its bases and
and 2 heights values (minimum and maximum allowed flight heights of the drone),
defining the bases vertical location (along Z-axis) in space.
The borders of the polygonal prism are treaded as walls of obstacles.
The base polygon is defined by the number of vertices and their XY-location.
As the script is started, the user is asked to define the flight polygonal area.
Ones it is done, the drones takes off to the maximum allowed height and start exploration of the
previously defined flight volume. The exploration trajectory is defined by
coverage path planning algorithm (CPP). The main parameter to define a CPP
trajectory is sweep resolution, which defines the distance between Gamma-shaped
sub-trajectories as well as the height change, ones the polygonanl region
on a constant height is explored.
The algorithm is augmented with collision avoidance in the layered manner.
The CPP trajectory is treated as a global planner path, which is corrected by the local
planner aimed on obstacle avoidance and trajectory feasibility for the robot,
taking in account the linear and rotational speed limits.
The global planner trajectory is divided to a series of consequent waypoints.
The collision-free route between 2 consequetive waypoints is a task for the local
trajectory planner. The robot's heading and linear velocity are deterimend in
motion() function based on the distance to the goal, current robot state and
presence of obstacles in the region, percieved by the multiranger sensor.
"""
import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import numpy as np
from grid_based_sweep_coverage_path_planner import planning
from connect_crazyflie import reset_estimator
from connect_crazyflie import activate_high_level_commander
from connect_crazyflie import activate_mellinger_controller

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import time
from tqdm import tqdm

import rospy
from drone_multiranger import DroneMultiranger
from grid_map import GridMap
from tools import define_polygon, polygon_contains_point
from flight_functions import takeoff, land, goTo, hover, fly
from flight_functions import left_shift, right_shift, back_shift, forward_shift, turn_left, turn_right, slow_down


def prepare(drone):
    scf = drone.scf
    activate_high_level_commander(scf)
    reset_estimator(scf)
    activate_mellinger_controller(scf, False)

def plot_robot2D(pose, params):
    r = params.sensor_range_m
    plt.plot([pose[0]-r*np.cos(pose[2]), pose[0]+r*np.cos(pose[2])],
             [pose[1]-r*np.sin(pose[2]), pose[1]+r*np.sin(pose[2])], '--', color='b')
    plt.plot([pose[0]-r*np.cos(pose[2]+np.pi/2), pose[0]+r*np.cos(pose[2]+np.pi/2)],
             [pose[1]-r*np.sin(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)], '--', color='b')
    plt.plot(pose[0], pose[1], 'ro', markersize=5)
    plt.arrow(pose[0], pose[1], 0.05 * np.cos(pose[2]), 0.05 * np.sin(pose[2]),
              head_length=0.1, head_width=0.1)

def plot_robot(ax, pose, params):
    r = params.sensor_range_m
    ax.plot([pose[0]-r*np.cos(pose[3]), pose[0]+r*np.cos(pose[3])],
             [pose[1]-r*np.sin(pose[3]), pose[1]+r*np.sin(pose[3])],
             [pose[2], pose[2]], '--', linewidth=1, color='b')
    ax.plot([pose[0]-r*np.cos(pose[3]+np.pi/2), pose[0]+r*np.cos(pose[3]+np.pi/2)],
             [pose[1]-r*np.sin(pose[3]+np.pi/2), pose[1]+r*np.sin(pose[3]+np.pi/2)],
             [pose[2], pose[2]], '--', linewidth=1, color='b')
    ax.scatter(pose[0], pose[1], pose[2], marker='^')
    ax.quiver(pose[0], pose[1], pose[2], np.cos(pose[3]), np.sin(pose[3]), 0.0, length=0.2, normalize=True)


def border_check(pose, gridmap, params):
    gmap = gridmap

    r = int(100*params.sensor_range_m)
    back = [pose[0]-r*np.cos(pose[2]), pose[1]-r*np.sin(pose[2])]
    front = [pose[0]+r*np.cos(pose[2]), pose[1]+r*np.sin(pose[2])]
    right = [pose[0]+r*np.cos(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)]
    left = [pose[0]-r*np.cos(pose[2]+np.pi/2), pose[1]-r*np.sin(pose[2]+np.pi/2)]

    pi = np.array(pose[:2], dtype=int)
    backi = np.array(back, dtype=int)
    fronti = np.array(front, dtype=int)
    lefti = np.array(left, dtype=int)
    righti = np.array(right, dtype=int)

    obstacle = {
        'front': 0,
        'back':  0,
        'right': 0,
        'left':  0,
    }

    for i in np.arange(min(pi[0], fronti[0]), max(pi[0], fronti[0])+1):
        for j in np.arange(min(pi[1], fronti[1]), max(pi[1], fronti[1])+1):
            m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
            if gmap[m,n]:
                # print('FRONT collision')
                obstacle['front'] = 1

    for i in np.arange(min(pi[0], backi[0]), max(pi[0], backi[0])+1):
        for j in np.arange(min(pi[1], backi[1]), max(pi[1], backi[1])+1):
            m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
            if gmap[m,n]:
                # print('BACK collision')
                obstacle['back'] = 1

    for i in np.arange(min(pi[0], lefti[0]), max(pi[0], lefti[0])+1):
        for j in np.arange(min(pi[1], lefti[1]), max(pi[1], lefti[1])+1):
            m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
            if gmap[m,n]:
                # print('LEFT collision')
                obstacle['left'] = 1

    for i in np.arange(min(pi[0], righti[0]), max(pi[0], righti[0])+1):
        for j in np.arange(min(pi[1], righti[1]), max(pi[1], righti[1])+1):
            m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
            if gmap[m,n]:
                # print('RIGHT collision')
                obstacle['right'] = 1

    return obstacle

def motion(state, goal, params):
    """
    Defines motion model of the robot with the following state variables:
    state = [x(m), y(m), z(m), yaw(rad), v(m/s), omega(rad/s)]
    """
    dx = goal[0] - state[0]
    dy = goal[1] - state[1]
    goal_yaw = math.atan2(dy, dx)
    K_theta = 0.5
    dyaw = K_theta*math.sin(goal_yaw - state[3])
    # dyaw = K_theta*np.sign(goal_yaw - state[3])*np.abs(goal_yaw - state[3])
    # if dyaw >= 0.2: dyaw = 0.2
    # if dyaw <= -0.2: dyaw = -0.2

    state[5] = dyaw / params.dt  # omega(rad/s)
    state[3] += dyaw # yaw(rad)

    dist_to_goal = np.linalg.norm(goal - state[:3])
    K_xy = 0.3
    dp = K_xy*dist_to_goal
    if dp >= params.max_vel * params.dt: dp = params.max_vel * params.dt
    if dp <= params.min_vel * params.dt: dp = params.min_vel * params.dt

    state[4] = dp / params.dt

    state[0] += dp*np.cos(state[3]) # x(m)
    state[1] += dp*np.sin(state[3]) # y(m)

    dist_to_goalZ = np.linalg.norm(goal[2] - state[2])
    K_z = 0.5
    dz = K_z * dist_to_goalZ
    if dz >= 0.1: dz = 0.1

    state[2] += dz * np.sign(goal[2] - state[2])

    return state

def avoid_borders(state, gridmap, params):
    pose_grid = gridmap.meters2grid(state[:2])
    yaw = state[3]
    boundary = border_check([pose_grid[0], pose_grid[1], yaw], gridmap.gmap, params)
    # print(boundary)

    if boundary['right'] or boundary['front']:
        print('Border FRONT RIGHT')
        # state = back_shift(state, 0.03)
        state = slow_down(state, params.min_vel, dv=0.02)
        state = turn_left(state, np.radians(20))
        # state = forward_shift(state, 0.02)
    elif boundary['left']:
        print('Border LEFT')
        # state = back_shift(state, 0.03)
        state = slow_down(state, params.min_vel, dv=0.02)
        state = turn_right(state, np.radians(20))
        # state = forward_shift(state, 0.02)
    return state

def avoid_obstacles(drone, params):
    def is_close(measured_range):
        # MIN_DISTANCE = 1000*params.sensor_range_m # mm
        MIN_DISTANCE = 350 # mm
        if measured_range is None:
            return False
        else:
            return measured_range < MIN_DISTANCE

    if is_close(drone.measurement['front']) and drone.measurement['left'] > drone.measurement['right']:
        print('FRONT RIGHT')
        drone.state = slow_down(drone.state, params.min_vel, dv=0.05)
        drone.state = back_shift(drone.state, 0.04)
        drone.state = turn_left(drone.state, np.radians(20))
        drone.state = forward_shift(drone.state, 0.04)
    if is_close(drone.measurement['front']) and drone.measurement['left'] < drone.measurement['right']:
        print('FRONT LEFT')
        drone.state = slow_down(drone.state, params.min_vel, dv=0.04)
        drone.state = back_shift(drone.state, 0.04)
        drone.state = turn_right(drone.state, np.radians(20))
        drone.state = forward_shift(drone.state, 0.04)
    if is_close(drone.measurement['left']):
        print('LEFT')
        drone.state = right_shift(drone.state, 0.04)
    if is_close(drone.measurement['right']):
        print('RIGHT')
        drone.state = left_shift(drone.state, 0.04)


def flight_mission(drone, goal_x, goal_y, goal_z, params):
    if params.toFly:
        takeoff(drone, params.max_height)
    goal = [goal_x[drone.goali], goal_y[drone.goali], goal_z[drone.goali]] # goal = [x, y, z], m
    # initial state = [x(m), y(m), z(m), yaw(rad), v(m/s), omega(rad/s)]
    drone.state = np.array([drone.position[0], drone.position[1], drone.position[2],
                            0.0, 0.0, 0.0])
    drone.traj = drone.state[:3]
    t_prev_goal = time.time()

    # while True:
    # for _ in tqdm( range(params.numiters) ):
    for _ in range(params.numiters):
        drone.state = motion(drone.state, goal, params)

        drone.state = avoid_borders(drone.state, gridmap, params)

        if params.toFly and params.collision_avoidance: avoid_obstacles(drone, params)

        goal_dist = np.linalg.norm(goal - drone.state[:3])
        # print('Distance to goal %.2f [m]:' %goal_dist)
        t_current = time.time()
        if goal_dist < params.goal_tol or (t_current - t_prev_goal) > params.time_to_switch_goal: # goal is reached
            # print('Switching to the next goal.')
            # print('Time from the previous reached goal:', t_current - t_prev_goal)
            if drone.goali < len(goal_x) - 1:
                drone.goali += 1
            else:
                drone.goali = 0
                break
            t_prev_goal = time.time()
            goal = np.array([goal_x[drone.goali], goal_y[drone.goali], goal_z[drone.goali]])


        drone.sp = [drone.state[0], drone.state[1], drone.state[2], np.degrees(drone.state[3])%360]
        if params.toFly: fly(drone)
        drone.processing.publish_pose(drone.sp[:3], [0,0,0], 'drone_sp')

        drone.traj = np.vstack([drone.traj, drone.state[:3]])

        if params.check_battery:
            try:
                if drone.battery_state == 'needs_charging':
                    print('Going home to CHARGE the battery')
                    print('Battery status: %.2f [V]' %drone.V_bat)
                    break
            except:
                pass

        if params.toFly:
            time.sleep(0.1)
        elif params.animate:
            plt.cla()
            ax.set_xlabel('X, [m]')
            ax.set_ylabel('Y, [m]')
            ax.set_zlabel('Z, [m]')
            ax.set_xlim([-1.5, 1.5])
            ax.set_ylim([-1.5, 1.5])
            ax.set_zlim([0.0, 2.0])
            ax.plot(goal_x, goal_y, goal_z, ':')
            ax.scatter(goal[0], goal[1], goal[2], label='Goal position', zorder=20)
            ax.plot(drone.traj[:,0], drone.traj[:,1], drone.traj[:,2], linewidth=3, color='g')
            plot_robot(ax, drone.state, params)
            plt.legend()
            plt.pause(0.1)

    if params.toFly:
        print('Going home along Pi-shaped trajectory')
        goTo(drone, [drone.position[0], drone.position[1], 0.8, 0])
        hover(drone, 1.0)
        goTo(drone, [drone.pose_home[0], drone.pose_home[1], 0.8, 0])
        hover(drone, 1.0)
        goTo(drone, [drone.pose_home[0], drone.pose_home[1], drone.pose_home[2]+0.1, 0])
        hover(drone, 1.0)
        land(drone, height=drone.pose_home[2]+0.04)


def exploration_conveyer(drone, goal_x, goal_y, goal_z, params):
    """
    The exploration conveyer monitors the battery charge of the drone.
    It doesn't allow a drone to take off, if its battery is not charged enough.
    It also handles repitability of the flight mission. Ones the flight mission
    is completed (or partially completed), the exploration conveyer checks if
    the drone is landed properly on its home location (with wireless charger).
    If the drone is not accurately landed on the wireless charger it will
    be given a task to take off on a small height, correct its position
    relative to its home location and try to land properly again in order
    wireless charging process is started.
    """
    def land_to_charge(drone, params):
        for _ in range(params.land_to_charge_attempts):
            if drone.charging_state != 1:
                takeoff(drone, drone.pose_home[2]+0.2)
                goTo(drone, [drone.pose_home[0], drone.pose_home[1], drone.pose_home[2]+0.15, 0.0])
                hover(drone, t_hover=4)
                land(drone, height=drone.pose_home[2]+0.0, land_speed=0.05)
                time.sleep(4)
            else:
                print('Charging started')
                break
    # while True:
    for _ in range(params.num_missions):
        if params.check_battery:
            # Waiting for the battery to become charged
            while True:
                try:
                    if (drone.battery_state == 'fully_charged'):
                        print('Battery status: %.2f [V]' %drone.V_bat)
                        break
                except:
                    pass
        # One flight mission
        print("Starting the mission!")
        flight_mission(drone, goal_x, goal_y, goal_z, params)
        time.sleep(4)
        if params.toFly: land_to_charge(drone, params)
        time.sleep(params.time_between_missions)

def define_flight_area(initial_pose, params):
    plt.grid()
    plot_robot2D(initial_pose, params)
    while True:
        try:
            num_pts = int( input('Enter number of polygonal vertixes: ') )
            break
        except:
            print('\nPlease, enter an integer number.')
    while True:
        flight_area_vertices = define_polygon(num_pts)
        if polygon_contains_point(initial_pose[:2], flight_area_vertices):
            break
        plt.clf()
        plt.grid()
        plot_robot2D(initial_pose, params)
        print('The robot is not inside the flight area. Define again.')
    return flight_area_vertices

def get_3D_waypoints(goal_x, goal_y, h_min, h_max, dh):
    height_levels = np.linspace( h_max, h_min, int((h_max-h_min)/dh) )
    waypoints = np.vstack([goal_x, goal_y, height_levels[0]*np.ones_like(goal_x)])
    for i in range(1, len(height_levels)):
        level_x = np.copy(goal_x)
        level_y = np.copy(goal_y)
        if i%2 == 1:
            level_x = np.flip(goal_x)
            level_y = np.flip(goal_y)
        level_z = height_levels[i]*np.ones_like(goal_x)
        waypoints1 = np.vstack([level_x, level_y, level_z])
        waypoints = np.hstack([waypoints, waypoints1])
    return waypoints.T # waypoints = [goal_x.T, goal_y.T, height_levels.T]


class Params:
    def __init__(self):
        self.numiters = 1000
        self.uri = 'radio://0/80/2M/E7E7E7E702'
        self.toFly = 1
        self.check_battery = 0 # doesn't work properly
        self.animate = 0
        self.dt = 0.1
        self.goal_tol = 0.15
        self.sweep_resolution = 0.25
        self.max_vel = 0.15 # m/s
        self.min_vel = 0.1 # m/s
        self.max_height = 0.6 # m
        self.min_height = 0.3 # m
        self.sensor_range_m = 0.20 # m
        self.time_to_switch_goal = 5.0 # sec
        self.land_to_charge_attempts = 0
        self.num_missions = 1
        self.time_between_missions = 5
        self.collision_avoidance = True


if __name__ == '__main__':
    rospy.init_node('random_walk')
    params = Params()
    drone = DroneMultiranger(params.uri)
    time.sleep(3)
    drone.goali = 0
    drone.pose_home = drone.position
    # drone.pose_home = [0,0,0]
    print('Home positions:', drone.pose_home)

    flight_area_vertices = define_flight_area(drone.pose_home, params)
    # SCALE = 1.5; flight_area_vertices = SCALE * np.array([[-0.6, 0.8], [-0.9, -0.9], [0.8, -0.8], [0.5, 0.9]])
    gridmap = GridMap(flight_area_vertices, drone.pose_home[:2])

    ox = flight_area_vertices[:,0].tolist() + [flight_area_vertices[0,0]]
    oy = flight_area_vertices[:,1].tolist() + [flight_area_vertices[0,1]]

    goal_x2D, goal_y2D = planning(ox, oy, params.sweep_resolution)

    waypoints = get_3D_waypoints(goal_x2D, goal_y2D, params.min_height, params.max_height, params.sweep_resolution/2.)
    goal_x = waypoints[:,0]
    goal_y = waypoints[:,1]
    goal_z = waypoints[:,2]

    if params.toFly:
        prepare(drone)
        raw_input('Press Enter to fly...')

    fig = plt.figure(figsize=(10,10))
    ax = plt.axes(projection='3d')

    exploration_conveyer(drone, goal_x, goal_y, goal_z, params)

    plt.cla()
    ax.plot(goal_x, goal_y, goal_z, ':')
    ax.plot(drone.traj[:,0], drone.traj[:,1], drone.traj[:,2], linewidth=3, color='g')
    plt.legend()
    plt.pause(0.1)
    raw_input('Hit Enter to close all figures.')
    plt.close('all')

    print('Mission is complete!')
    drone.disconnect()
    

