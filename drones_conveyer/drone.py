
import time
import numpy as np
from numpy.linalg import norm

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger


V_BATTERY_TO_GO_HOME = 3.4
V_BATTERY_CHARGED = 3.7

class Drone:
    def __init__(self, uri='radio://0/80/2M'):
        self.uri = uri # URI to the Crazyflie to connect to
        self.cf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')).cf
        self.pose = None
        self.pose_home = np.array([0,0,0])
        self.orient = None
        self.sp = None
        self.battery_state = ''

    def fly(self):
        self.cf.commander.send_position_setpoint(self.sp[0], self.sp[1], self.sp[2], self.sp[3])

    def takeoff(self, height=0.3):
        # takeoff to z=0.3 m:
        print('Takeoff...')
        self.sp = np.zeros(4); self.sp[:3] = self.pose
        dz = 0.02
        for i in range(int(height/dz)):
            self.sp[2] += dz
            self.fly()
            time.sleep(0.1)

    def land(self):
        print('Landing...')
        while self.sp[2]>-0.1:
            self.sp[2] -= 0.02
            self.fly()
            time.sleep(0.1)
        self.stop()
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
    def stop(self):
        self.cf.commander.send_stop_setpoint()

    def goTo(self, goal, vel=0.3, pos_tol=0.03, yaw_tol=3):
        goal = np.array(goal)
        print('Going to', goal)
        if self.sp is None:
            self.sp = np.zeros(4); self.sp[:3] = self.pose
        dt = 0.1
        while norm(goal[:3] - self.sp[:3]) > pos_tol or norm(self.sp[3]-goal[3]) > yaw_tol:
            n = normalize(goal[:3] - self.sp[:3])
            self.sp[:3] += dt*vel * n # position setpoints
            self.sp[3] += 3 * np.sign( goal[3] - self.sp[3] ) # yaw angle
            # print('Yaw', self.sp[3], 'yaw diff', norm(self.sp[3]-goal[3]))
            self.fly()
            time.sleep(dt)

    def hover(self, t_hover=2):
        t0 = time.time()
        while time.time() - t0 < t_hover:
            self.fly()
            time.sleep(0.1)

    def trajectory(self):
        """ Figure 8 trajectory """
        # 1-st circle
        for _ in range(50):
            self.cf.commander.send_hover_setpoint(0.5, 0, 36 * 2, 1.3)
            time.sleep(0.1)
        # 2-nd circle
        for _ in range(50):
            self.cf.commander.send_hover_setpoint(0.5, 0, -36 * 2, 1.3)
            time.sleep(0.1)
        # hover for 2 sec
        for _ in range(20):
            self.cf.commander.send_hover_setpoint(0, 0, 0, 1.3)
            time.sleep(0.1)
    def trajectory_battery_check(self):
        """ Figure 8 trajectory """
        # 1-st circle
        for _ in range(50):
            if not self.battery_state == 'needs_charging':
                self.cf.commander.send_hover_setpoint(0.5, 0, 36 * 2, 1.3)
                time.sleep(0.1)
        # 2-nd circle
        for _ in range(50):
            if not self.battery_state == 'needs_charging':
                self.cf.commander.send_hover_setpoint(0.5, 0, -36 * 2, 1.3)
                time.sleep(0.1)
        # hover for 2 sec
        for _ in range(20):
            if not self.battery_state == 'needs_charging':
                self.cf.commander.send_hover_setpoint(0, 0, 0, 1.3)
                time.sleep(0.1)

    def position_callback(self, timestamp, data, logconf):
        x = data['kalman.stateX']
        y = data['kalman.stateY']
        z = data['kalman.stateZ']
        self.pose = np.array([x, y, z])
        roll = np.radians( data['stabilizer.roll'] )
        pitch = np.radians( data['stabilizer.pitch'] )
        yaw = np.radians( data['stabilizer.yaw'] )
        self.orient = np.array([roll, pitch, yaw])
    def start_position_reading(self):
        log_conf = LogConfig(name='Position', period_in_ms=50) # read position with 20 Hz rate
        log_conf.add_variable('kalman.stateX', 'float')
        log_conf.add_variable('kalman.stateY', 'float')
        log_conf.add_variable('kalman.stateZ', 'float')
        log_conf.add_variable('stabilizer.roll', 'float')
        log_conf.add_variable('stabilizer.pitch', 'float')
        log_conf.add_variable('stabilizer.yaw', 'float')
        self.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.position_callback)
        log_conf.start()

    def battery_callback(self, timestamp, data, logconf):
        self.V_bat = data['pm.vbat']
        # print('Battery status: %.2f [V]' %self.V_bat)
        if self.V_bat <= V_BATTERY_TO_GO_HOME:
            self.battery_state = 'needs_charging'
            # print('Battery is not charged: %.2f' %self.V_bat)
        elif self.V_bat >= V_BATTERY_CHARGED:
            self.battery_state = 'fully_charged'
    def start_battery_status_reading(self):
        log_conf = LogConfig(name='Battery', period_in_ms=500) # read battery status with 2 Hz rate
        log_conf.add_variable('pm.vbat', 'float')
        self.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.battery_callback)
        log_conf.start()


""" Helper functions """
def normalize(vector):
    vector = np.array(vector)
    v_norm = vector / norm(vector) if norm(vector)!=0 else np.zeros_like(vector)
    return v_norm
