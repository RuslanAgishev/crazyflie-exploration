import numpy as np
import time


def takeoff(drone, height=0.3):
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
    # print(sp)
    drone.cf.commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
def land(drone, height=-0.1, land_speed=0.2):
    print('Landing...')
    while drone.sp[2]>height:
        drone.sp[2] -= land_speed*0.1
        fly(drone)
        time.sleep(0.1)
    drone.cf.commander.send_stop_setpoint()
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
    yaw = pose[3]
    left = [pose[0]+r*np.cos(yaw+np.pi/2), pose[1]+r*np.sin(yaw+np.pi/2)]
    pose[:2] = left
    return np.array(pose)
def right_shift(pose, r):
    yaw = pose[3]
    right = [pose[0]-r*np.cos(yaw+np.pi/2), pose[1]-r*np.sin(yaw+np.pi/2)]
    pose[:2] = right
    return np.array(pose)
def back_shift(pose, r):
	yaw = pose[3]
	back = pose
	back[:2] = [pose[0]-r*np.cos(yaw), pose[1]-r*np.sin(yaw)]
	return back
def forward_shift(pose, r):
	yaw = pose[3]
	forward = pose
	forward[:2] = [pose[0]+r*np.cos(yaw), pose[1]+r*np.sin(yaw)]
	return forward
def turn_left(pose, yaw=np.pi/2*np.random.uniform(0.2, 0.6)):
	pose[3] -= yaw
	return pose
def turn_right(pose, yaw=np.pi/2*np.random.uniform(0.2, 0.6)):
	pose[3] += yaw
	return pose
def slow_down(state, min_vel, dv=0.05):
	if state[4]>min_vel:
		state[4] -= dv
	return state