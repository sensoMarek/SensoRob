# Sensorob IK tests package

## General
This package performs IK solver tests.

IK solver is selected in `sensorob_moveit_config` package in file `kinematics.yaml`. Available IK algorithms:
- IKFast
- KDL and KDL Cached
- TracIK

## Description
We created IK solver tests as followed:

**Forward Kinematics (FK)**
- Compute forward kinematics for a robot state
- Check robot's state validity (self-collisions checking)
- If valid, get robot's joint positions
- Save these positions to file `joins.csv` and save robot's pose (pose of link_6) to file `position.csv`

**Inverse Kinematics (IK)**
- Load poses from file `position.csv`
- In a loop compute IK task for given pose
- If solution found, save accuracy and computation time to file `accuracy_and_time.csv`
- If solution not found, save `-1` to a file `accuracy_and_time.csv` on a place of accuracy and time 
so user knows which poses were not computed successfully

**Visualization**
- Load poses from file `position.csv` and select only point (x,y,z) translation for each pose
- Visualize all points from file
- Visualize missing points (which IK solver did not find)
- Visualize all points on xy-plane
- Visualize all points on yz-plane
- Visualize all points on xz-plane

<p align="center">
<img src="/media/images/ik_test_viz_all_points.png" width="40%" title="Visualized all point used for IK solver test">
<img src="/media/images/ik_test_viz_space.png" width="40%" title="Visualized selection of robot's configuration space ">
</p>

---

We also created 2 ways of generating poses in FK.
**1. Joint states sampling**
- We load joint limits for each joint (except 6th) and then compute intervals for each joint
- Parameter `num_of_joint_samples` tells how many samples user want to create on the interval for each joint
- Then we interpolate intervals by this number and create set of discrete states of joint positions
- We loop through each combination of joint states and create robot states

**2. Random robot states**
- By using `setToRandomPositions` function we get random position in configuration space of the robot
- Then we return also joint states
- Parameter `num_of_samples` tells how many valid states FK should save to a file

---

## Launch
The simulation has to be launched first, e.g.   
```
ros2 launch sensorob moveit_mock.launch.py
```

Then the IK solver test is ready to run  (run in second terminal)
```
ros2 launch sensorob_ik_interface ik_interface.launch.py
```

or   
```
ros2 launch sensorob_ik_interface ik_interface_random.launch.py
```

Use `-s` argument to see all the arguments, for instance:  
```
ros2 launch sensorob_ik_interface ik_interface.launch.py -s
```

```
Arguments (pass arguments as '<name>:=<value>'):

    'num_of_joint_samples':
        Number of joint samples.
This number will represent how many total samples will be used for IK test given formula num_of_joint_samples^(num of used joints)
        (default: '6')

    'computeIK':
        If False, inverse kinematics will be not computed
        (default: 'True')

    'computeFK':
        If False, forward kinematics will be not computed
        (default: 'True')

    'logs_folder_path':
        The path to the log folder in your PC, where files will be stored when performing IK test
        (default: '/home/jakub/ros2_ws/src/SensoRob/sensorob_logs/ik')
```

During the test there are prints (hints) in second terminal where ik_interface was launched about progress.
Also `Press 'next' in the RvizVisualToolsGui window to ...` is printed, so user can sequentially continue by clicking on the button in RViZ.