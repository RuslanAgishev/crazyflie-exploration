# HSS-Uley

## Pre-flight setup
Instructions on how to setup drones and lighthouse localization system are available via the [link](https://docs.google.com/document/d/1FPMGKbiM-bM_8lxtKZ4foJDvCfnwSi7CRuANoXPdRxE/edit?usp=sharing).
Crazyflie firmware is accessible [here](https://drive.google.com/file/d/1wb32-55Z5RYd08ETg4leyU9her5YEeub/view?usp=sharing). Follow the [instructions](https://github.com/bitcraze/crazyflie-firmware) on how to build and flash the firmware.

## Flight scripts description
### Python scripts without ROS
Drones sequential flight in a conveyer manner followed by a syncronized mission:
```bash
source drones_conveyer/conveyer_and_swarm.sh 
```
### Using ROS for data exchange and visualization
In general you need to execute to commands, one to initialize a visualization environment and another to launch the drones.
Launch simulated environment:
```bash
roslaunch cf_inspection map.launch
```
In order to run cf_inscpection ROS node scripts start master and launch helper visualization tools:
```bash
roslaunch cf_inspection multiranger_map.launch
```
And then execute one of the flight scripts, for example:
```bash
rosrun cf_inspection random_walk.py
```
