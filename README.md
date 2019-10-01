# HSS-Uley

<!--
Drones sequential flight in a conveyer manner followed by a syncronized mission:
```bash
source drones_conveyer/conveyer_and_swarm.sh 
```
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
-->
