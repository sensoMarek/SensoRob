## Sensorob description

- ${\textsf{\color{#ff5c64}config}}$
    - contains configuration file for each joint, example:

    ```
    # drive 5012 52Nm, 16/min-1
    joint_1:
    upper: 1.05  # 45deg 
    lower: -1.05  # -45deg 
    velocity: 0.65  # 1.6755 * 0.4
    effort: 52
    dumping: 0.001
    friction: 0.001
    init_pos: 0.0
    init_vel: 0.0
    ```

- ${\textsf{\color{#ff5c64}meshes}}$
    - contains all robot meshes creating 3d model of the robot

- ${\textsf{\color{#ff5c64}urdf}}$
    - contains <b>[robot core](urdf/robot_core.xacro)</b> describing robot model
    - contains <b>[ros2 control](urdf/ros2_control.xacro)</b> describing joints and its interfaces. This file is separated into 2 sections. 
    - First section describes joints for simulated system, whereas the second one aims for real system using ethercat driver for ros2. Each joint is configured as an EtherCAT module using provided configuration file and mode of operation. Also state and comand interfaces are defined there. Also position on ethercat bus is configured along with aliases (not used in this case, therefore set to 0 for all joints)
        
        Example:
        ```
                    <joint name="joint_1">
                <command_interface name="position"/>
                <command_interface name="velocity"/>
                <command_interface name="effort"/>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
                <state_interface name="effort"/>
                <ec_module name="senso_joint_1">
                    <plugin>ethercat_generic_plugins/EcSensoDrive</plugin>
                    <param name="alias">0</param>
                    <param name="position">0</param>
                    <param name="mode_of_operation">-108</param>
                    <param name="slave_config">$(find sensorob)/config/joints_driver_config/senso_joint_1_advanced.yaml</param>
                </ec_module>
        ```