#!/usr/bin/env python

"""
Random walk algorithm implementation for a mobile robot
equipped with 4 ranger sensors (front, back, left and right)
for obstacles detection

author: Ruslan Agishev (agishev_ruslan@mail.ru)
reference: https://ieeexplore.ieee.org/abstract/document/6850799/s
"""

import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.patches import Polygon
import numpy as np

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

import rospy
from multiranger import DroneMultiranger
import time
from threading import Thread


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


def plot_arrow(x, y, yaw, length=0.1, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)

def plot_robot(pose, params):
	yaw = pose[2]
	r = int(params.proximity_to_virtual_walls_m)
	plt.plot([pose[0]-r*np.cos(yaw), pose[0]+r*np.cos(yaw)],
			 [pose[1]-r*np.sin(yaw), pose[1]+r*np.sin(yaw)], '--', color='b')
	plt.plot([pose[0]-r*np.cos(yaw+np.pi/2), pose[0]+r*np.cos(yaw+np.pi/2)],
		     [pose[1]-r*np.sin(yaw+np.pi/2), pose[1]+r*np.sin(yaw+np.pi/2)], '--', color='b')
	plt.plot(pose[0], pose[1], 'ro', markersize=5)
	plot_arrow(pose[0], pose[1], yaw)

def borders_check(pose, params):
	gmap = params.gmap

	r = int(100*params.proximity_to_virtual_walls_m)
	back = [pose[0]-r*np.cos(pose[2]), pose[1]-r*np.sin(pose[2])]
	front = [pose[0]+r*np.cos(pose[2]), pose[1]+r*np.sin(pose[2])]
	right = [pose[0]+r*np.cos(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)]
	left = [pose[0]-r*np.cos(pose[2]+np.pi/2), pose[1]-r*np.sin(pose[2]+np.pi/2)]

	pi = np.array(pose[:2], dtype=int)
	backi = np.array(back, dtype=int)
	fronti = np.array(front, dtype=int)
	lefti = np.array(left, dtype=int)
	righti = np.array(right, dtype=int)

	border = {
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
				border['front'] = 1

	for i in np.arange(min(pi[0], backi[0]), max(pi[0], backi[0])+1):
		for j in np.arange(min(pi[1], backi[1]), max(pi[1], backi[1])+1):
			m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
			if gmap[m,n]:
				# print('BACK collision')
				border['back'] = 1

	for i in np.arange(min(pi[0], lefti[0]), max(pi[0], lefti[0])+1):
		for j in np.arange(min(pi[1], lefti[1]), max(pi[1], lefti[1])+1):
			m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
			if gmap[m,n]:
				# print('LEFT collision')
				border['left'] = 1

	for i in np.arange(min(pi[0], righti[0]), max(pi[0], righti[0])+1):
		for j in np.arange(min(pi[1], righti[1]), max(pi[1], righti[1])+1):
			m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
			if gmap[m,n]:
				# print('RIGHT collision')
				border['right'] = 1
	return border

def meters2grid(pose_m, params):
    # [0, 0](m) -> [100, 100]
    # [1, 0](m) -> [100+100, 100]
    # [0,-1](m) -> [100, 100-100]
    nrows = int(params.map_width_X_m / params.map_resolution_m)
    ncols = int(params.map_length_Y_m / params.map_resolution_m)
    if np.isscalar(pose_m):
        pose_on_grid = int( pose_m/params.map_resolution_m + ncols/2 )
    else:
        pose_on_grid = np.array( np.array(pose_m)/params.map_resolution_m +\
        						 np.array([ncols/2, nrows/2]) -\
        						 params.map_center/params.map_resolution_m, dtype=int )
    return pose_on_grid
def grid2meters(pose_grid, params):
    # [100, 100] -> [0, 0](m)
    # [100+100, 100] -> [1, 0](m)
    # [100, 100-100] -> [0,-1](m)
    nrows = int(params.map_width_X_m / params.map_resolution_m)
    ncols = int(params.map_length_Y_m / params.map_resolution_m)
    if np.isscalar(pose_grid):
        pose_meters = (pose_grid - ncols/2) * params.map_resolution_m
    else:
        pose_meters = ( np.array(pose_grid) - np.array([ncols/2, nrows/2]) ) *\
        			    params.map_resolution_m - params.map_center
    return pose_meters

def visualize(drone):
	ax = plt.gca()
	ax.set_xlim([-2.5, 2.5])
	ax.set_ylim([-2.5, 2.5])
	w = drone.gridmap_params.map_length_Y_m
	l = drone.gridmap_params.map_width_X_m
	c = drone.gridmap_params.map_center
	boundaries = np.array([ c+[-w/2., -l/2.], c+[-w/2., +l/2.], c+[+w/2., +l/2.], c+[+w/2., -l/2.] ])
	ax.add_patch( Polygon(boundaries, linewidth=2, edgecolor='k',facecolor='none') )
	plt.plot(drone.traj[:,0], drone.traj[:,1], 'g')
	plot_robot(drone.pose, drone.gridmap_params)


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
    drone.cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)    
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


def left_shift(pose, r):
	left = pose
	left[:2] = [pose[0]+r*np.cos(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)]
	return left
def right_shift(pose, r):
	right = pose
	right[:2] = [pose[0]-r*np.cos(pose[2]+np.pi/2), pose[1]-r*np.sin(pose[2]+np.pi/2)]
	return right
def back_shift(pose, r):
	back = pose
	back[:2] = [pose[0]-r*np.cos(pose[2]), pose[1]-r*np.sin(pose[2])]
	return back

def is_close(range):
    MIN_DISTANCE = 350 # mm
    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


def exploration_mission(drone, params):
	drone.pose_home = drone.position
	print('Home positions:', drone.pose_home)

	if params.toFly: takeoff(drone, params.flight_height)

	#    x,    y,      yaw
	pose = [drone.pose_home[0], drone.pose_home[1], 0.0]
	traj = pose[:2]

	for _ in range(params.numiters):
		dv = 0.1*params.vel
		pose[0] += dv*np.cos(pose[2])
		pose[1] += dv*np.sin(pose[2])

		pose_grid = meters2grid(pose[:2], drone.gridmap_params)
		border = borders_check([pose_grid[0], pose_grid[1], pose[2]], drone.gridmap_params)
		# print(border)

		""" Virtual borders of the inspection area """
		if border['right'] or border['front']:
			# print('Border in FRONT or RIGHT')
			pose = back_shift(pose, 0.05) # go back
			pose[2] -= np.pi/2 * np.random.uniform(0.2, 0.6) # turn left
		elif border['left']:
			# print('Border on the LEFT')
			pose = back_shift(pose, 0.05) # go back
			pose[2] += np.pi/2 * np.random.uniform(0.2, 0.6) # turn right

		if params.toFly:
			""" Actions on multiranger sensor data """
			if is_close(drone.measurement['front']) and drone.measurement['left'] > drone.measurement['right']:
				# print('FRONT RIGHT')
				pose = back_shift(pose, 0.05)
				pose[2] += np.pi/2 * np.random.uniform(0.2, 0.5)
			if is_close(drone.measurement['front']) and drone.measurement['left'] < drone.measurement['right']:
				# print('FRONT LEFT')
				pose = back_shift(pose, 0.05)
				pose[2] += np.pi/2 * np.random.uniform(0.2, 0.5)
			if is_close(drone.measurement['left']):
				# print('LEFT')
				pose = right_shift(pose, 0.05)
			if is_close(drone.measurement['right']):
				# print('RIGHT')
				pose = left_shift(pose, 0.05)

		traj = np.vstack([traj, pose[:2]])
		drone.traj = traj
		
		drone.sp = [pose[0], pose[1], params.flight_height, np.degrees(pose[2])%360]
		drone.pose = pose

		try:
			if drone.battery_state == 'needs_charging':
				print('Going home to CHARGE the battery')
				break
		except:
			pass

		if params.toFly:
			fly(drone)
			time.sleep(0.1)

	if params.toFly:
		goTo(drone, [drone.pose_home[0], drone.pose_home[1], params.flight_height, 0])
		hover(drone, 1.0)
		land(drone)
	
	
class GridMap:
	def __init__(self, map_center=np.array([0.0, 0.0]), map_size=[2.0, 2.0]):
		self.map_center = map_center
		self.map_resolution_m = 0.01
		self.map_width_X_m = map_size[0]
		self.map_length_Y_m = map_size[1]
		self.proximity_to_virtual_walls_m = 0.1 # [m], virtual range sensor model for borders sensing
		self.wall_thickness_m = 1.0*self.proximity_to_virtual_walls_m
		self.create_borders_grid_map()

	def create_borders_grid_map(self):
		WIDTH = int(100 * (self.map_width_X_m))
		LENGTH = int(100 * (self.map_length_Y_m))
		border = int(100 * self.wall_thickness_m)
		gmap = np.zeros([WIDTH, LENGTH])
		# walls
		gmap[:border, :] = 1
		gmap[-border:, :] = 1
		gmap[:, :border] = 1
		gmap[:, -border:] = 1
		self.gmap = gmap


class Params:
	def __init__(self):
		self.numiters = 300
		self.vel = 0.2 # [m/s]
		self.uris = [
					# 'radio://0/80/2M/E7E7E7E701',
					'radio://0/80/2M/E7E7E7E702',
					'radio://0/80/2M/E7E7E7E703',
					]
		self.flight_height = 0.2 # [m]
		self.toFly = 1


def main():
	rospy.init_node('swarm_random_walk')
	params = Params()
	# Connect to drones and start multirangers:
	drones = []
	for URI in params.uris:
		drone = DroneMultiranger(URI)
		drones.append(drone)
	time.sleep(3)

	# Define flight zones for each UAV:
	drones[0].gridmap_params = GridMap(map_center=np.array([0.0, -0.5]), map_size=[1.0, 2.0])
	drones[1].gridmap_params = GridMap(map_center=np.array([0.0, 0.5]), map_size=[1.0, 2.0])

	if params.toFly:
		th1 = Thread(target=prepare, args=(drones[0],) )
		th2 = Thread(target=prepare, args=(drones[1],) )
		th1.start(); th2.start();
		th1.join(); th2.join();

		raw_input('Press Enter to fly...')

	th1 = Thread(target=exploration_mission, args=(drones[0], params,) )
	th2 = Thread(target=exploration_mission, args=(drones[1], params,) )
	th1.start(); th2.start();
	th1.join(); th2.join();

	plt.figure(figsize=(10,10))
	for drone in drones:
		visualize(drone)
	plt.show()

if __name__ == '__main__':
	try:
		main()
	except KeyboardInterrupt:
		pass
		