
"""
The MotionCommander uses velocity setpoints.
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

import sys

if len(sys.argv)>1:
    URI = 'radio://0/80/2M/E7E7E7E7'+sys.argv[1][-2:]
else:
    URI = 'radio://0/80/2M/E7E7E7E702'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            time.sleep(1)

            # There is a set of functions that move a specific distance
            # We can move in all directions
            mc.up(0.3)
            time.sleep(2)

            mc.down(0.3)
            time.sleep(2)

            # And we can stop
            mc.stop()

            # We land when the MotionCommander goes out of scope
