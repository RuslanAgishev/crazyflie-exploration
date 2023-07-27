# Crazyflie Exploration
Swarm of drones flight scripts based on [crazyflie-lib-python](https://github.com/bitcraze/crazyflie-lib-python).

## Pre-flight setup
Instructions on how to setup drones and lighthouse localization system are available via the [link](https://docs.google.com/document/d/1FPMGKbiM-bM_8lxtKZ4foJDvCfnwSi7CRuANoXPdRxE/edit?usp=sharing).
Crazyflie firmware is accessible [here](https://drive.google.com/file/d/1wb32-55Z5RYd08ETg4leyU9her5YEeub/view?usp=sharing). Follow the [instructions](https://github.com/bitcraze/crazyflie-firmware) on how to build and flash the firmware.

## Flight scripts description

### Python scripts without ROS
Ones the setup is done, you can perform a test flight, moving a drone UP and DOWN:
```bash
python drone_test.py 
```
Note that you need to specity correct URI for the drone [here](https://github.com/Neolant-Uley/HSS-Uley/blob/master/drone_test.py#L19).

Drones sequential flight in a conveyer manner followed by a syncronized mission:
```bash
source drones_conveyer/conveyer_and_swarm.sh 
```

### Using ROS for data exchange and visualization
Before executing ROS scripts, build the workspace.:
```bash
cd ros_ws/
catkin_make
source devel/setup.bash
```
Note, that you need to do this only ones after you cloned the repository.

In general, you need to execute two commands, one to initialize a visualization environment and another to launch the drones.

Launch simulated environment. The following command brings up ROS master and starts helper visualization tools:
```bash
roslaunch cf_inspection multiranger_map.launch
```
or, if you want to add simulated objects to the environment, execute:
```bash
roslaunch cf_inspection map.launch
```
<img src="https://github.com/Neolant-Uley/HSS-Uley/blob/master/figures/map.png" width="400"/>

And then execute one of the flight scripts, for example:
```bash
rosrun cf_inspection random_walk.py
```

#### Flight scripts description
The scripts are located [here](https://github.com/Neolant-Uley/HSS-Uley/tree/master/ros_ws/src/cf_inspection/scripts) and distributed inside folders depending on their functions.

1. Flight missions in a conveyer manner:

   Simple example that connects to a crazyflie(check the URI address
   and update it to your crazyflie address) you use and send a flight mission
   to the drone. The battery charge is monitored during the flight. Ones the drone
   is discharged it will return home. This script intends to work with a wireless
   charging platform for the drone. Ones the drone is charged at its home position,
   it will continue the flight mission.
   ```bash
   rosrun cf_inspection misssion_battery_check.py
   ```

2. Pointcloud creation with multiranger sensors:

   <img src="https://github.com/Neolant-Uley/HSS-Uley/blob/master/figures/multiranger_map.png" width="400"/>
  
    Sprial demo. The group of 3 drones is initialy placed as follows:

                  1^         3^
 
                        0

                        2^
    where 0 denotes the origin, and numbers 1,2,3 labels of the
    crazyflies, ^ shows orientation along the X-axis.
    The swarm ascends via spiral to the maximum height of 1.6 m.
    The drones start flying one by one with a delay between neighbouring
    UAVs equal to 5.33 sec. Ones the maximum height is reached,
    each quadrotor performes a circular flight at a constant height
    and then starts to descend along spiral trajectory.
    One circular revolution consumes 8 sec of time.
    Total time of trajectory execution is 8x3 + 8 + 8x3 = 8x7 = 56 sec.
    ```bash
    rosrun cf_inspection swarm_multiranger_spiral.py
    ```
    
    Cylindrical demo. The group of 3 drones is again initialy placed as follows:

                  1^         3^
 
                        0

                        2^

    The swarm takes off at initial height, h0, and then
    the drones performe simultaneously circular trajectory at a constant height.
    After the full circle is executed, the heigt is increased with value dh.
    Then the swarm again performce a revolution at the new constant height.
    Ones the desired number of circular revolutions is reached (numiters), the drones start descending
    in the same manner.
    You can tune the specified parameters for your needs [here](https://github.com/Neolant-Uley/HSS-Uley/blob/master/ros_ws/src/cf_inspection/scripts/pointcloud_creation/swarm_multiranger_circle.py#L196). The flight demo is aimed at constructing a pointcloud
    with the help of multiranger or similar sensors of an object located at the origin inside the circle.
    ```bash
    rosrun cf_inspection swarm_multiranger_circle.py
    ```
 
3.  Exploration flights.

    Suppose, you need to perform an inspection task inside a specified flight area with the help of different sensors placed     on a drone.
    
    <img src="https://github.com/Neolant-Uley/HSS-Uley/blob/master/figures/random_walk.png" width="400"/>
    
    Random walk algorithm implementation for a robot (tested on Crazyflie 2.1)
    equipped with 4 ranger sensors (front, back, left and right)
    for obstacles detection. Flight area is defined as 
    a polygonal region by its vertixes. The robot should be placed inside the defined
    flight region. The region outside the flight area is treaded as a wall of obstacles.
    Ones the robot takes off, it travels forward until an obstacle is detected
    by the multiranger sensor or it approaches the wall closer, than the
    sensor sensitivity range. If obstacle or wall detection happens, the
    robot changes its flight direction, turning on a random angle. Note,
    that wall and obstacle detection algorithms are different and implemented as
    separate blocks.
    Take a look at the [paper](https://ieeexplore.ieee.org/abstract/document/6850799/) for reference.
    ```bash
    rosrun cf_inspection random_walk.py
    ```
    Check out the version ffor swarm of drones in specting different flight areas. You can specify desired areas [here](https://github.com/Neolant-Uley/HSS-Uley/blob/master/ros_ws/src/cf_inspection/scripts/exploration/random_walk/swarm_random_walk.py#L370).
    Note, that you again need to place each drone inside its predefined area before the flight.
    ```bash
    rosrun cf_inspection swarm_random_walk.py
    ```
    
4.  Coverage Path Planning (CPP).

    <img src="https://github.com/Neolant-Uley/HSS-Uley/blob/master/figures/cpp_3D_view.png" />
    
    Flight volume is defined as 
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
    sub-trajectories as well as the height change, ones the polygonal region
    on a constant height is explored.
    The algorithm is augmented with collision avoidance in the layered manner.
    The CPP trajectory is treated as a global planner path, which is corrected by the local
    planner aimed on obstacle avoidance and trajectory feasibility for the robot,
    taking in account the linear and rotational speed limits.
    The global planner trajectory is divided to a series of consequent waypoints.
    The collision-free route between 2 consequetive waypoints is a task for the local
    trajectory planner. The robot's heading and linear velocity are deterimend in
    [motion](https://github.com/Neolant-Uley/HSS-Uley/blob/master/ros_ws/src/cf_inspection/scripts/exploration/coverage_path_planning/cpp_3d.py#L132) function based on the distance to the goal, current robot state and
    presence of obstacles in the region, percieved by the multiranger sensor.
    
    ```bash
    rosrun cf_inspection cpp_3d.py
    ```
    
    If you need just a 2-dimensional CPP, try this example:
    ```bash
    rosrun cf_inspection cpp_const_height.py
    ```
    
    For more examples feel free to take a look at [motion-planning](https://github.com/RuslanAgishev/motion_planning) repository or [adaptive-swarm](https://github.com/RuslanAgishev/adaptive_swarm).
