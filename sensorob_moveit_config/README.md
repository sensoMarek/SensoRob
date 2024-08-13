## Sensorob Moveit Config

- ${\textsf{\color{#ff5c64}config}}$
    - stores configuration file used by the MoveIt2
    - [kinematics file](config/kinematics.yaml) defines which <b>Inverse kinematics</b> is used. We recommend IKFast or LMA
    - [stomp planning file](config/stomp_planning.yaml) holds variables for STOMP planner
- ${\textsf{\color{#ff5c64}launch}}$
    - launch files for demo + separate launch files launching components. We do not use them directly since in [sensorob package](../sensorob/README.md) we describe our custom launch files for several systems (kinematics, mocked, simulated, real)
