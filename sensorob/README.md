# Sensorob simulation package

## General
In config folder there is stored info about ROS2 Controllers.  
We are using `sensorob_group_controller` of type `joint_trajectory_controller/JointTrajectoryController`

The launch folder stores python launch files.

---
## Launch
### Kinematics  
In order to launch robot kinematics, type the following command in the terminal:
```
ros2 launch sensorob kinematics.launch.py
```

<p align="center">
<img src="/media/images/kinematics_screenshot.png" width="70%" title="Kinematics screenshot">
</p>

### Moveit + mock joint states
In order to launch the move group node with simulated mock joint states, type the following command in the terminal:
```
ros2 launch sensorob moveit_mock.launch.py 
```

<p align="center">
<img src="/media/images/moveit_mock_screenshot.png" width="70%" title="Moveit with mock joints screenshot">
</p>

### Moveit + simulated robot in Gazebo
In order to launch the move group node with simulated joint states published by robot model in Gazebo, type the following command in the terminal:
```
ros2 launch sensorob moveit_sim.launch.py 
```

<p align="center">
<img src="/media/images/moveit_sim_screenshot.png" width="70%" title="Moveit with Gazebo screenshot">
</p>