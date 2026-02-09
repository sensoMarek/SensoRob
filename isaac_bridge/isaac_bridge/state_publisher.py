import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState


class IsaacStatePublisher(Node):

    def __init__(self):
        super().__init__('isaac_state_publisher')
        self.get_logger().info('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX')
        topic = self.declare_parameter('isaac_topic', '/isaac/joint_states').get_parameter_value().string_value
        qos_depth = self.declare_parameter('qos_depth', 10).get_parameter_value().integer_value
        qos_profile = QoSProfile(depth=qos_depth)
        self.joint_names = list(
            self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
            .get_parameter_value().string_array_value
        )
        output_topic = self.declare_parameter('output_topic', '/joint_states').get_parameter_value().string_value
        self.joint_state_pub = self.create_publisher(JointState, output_topic, qos_profile)
        self.bridge_state_sub = self.create_subscription(
            JointState, topic, self.handle_state, qos_profile)
        self.mismatch_reported = False
        self.velocity_mismatch_reported = False
        self.effort_mismatch_reported = False
        self.get_logger().info(f'Bridging Isaac topic {topic} -> {output_topic}')

    def handle_state(self, msg: JointState) -> None:
        if len(msg.position) != len(self.joint_names):
            if not self.mismatch_reported:
                self.get_logger().warning(
                    f'Received {len(msg.position)} positions but expected {len(self.joint_names)}; discarding message'
                )
                self.mismatch_reported = True
            return
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = list(msg.position)

        if len(msg.velocity) == len(self.joint_names):
            joint_state.velocity = list(msg.velocity)
            if self.velocity_mismatch_reported:
                self.get_logger().info('Velocity length back in sync with joint_names')
                self.velocity_mismatch_reported = False
        elif msg.velocity:
            if not self.velocity_mismatch_reported:
                self.get_logger().warning(
                    f'Received {len(msg.velocity)} velocities but expected {len(self.joint_names)}; discarding velocities'
                )
                self.velocity_mismatch_reported = True

        if len(msg.effort) == len(self.joint_names):
            joint_state.effort = list(msg.effort)
            if self.effort_mismatch_reported:
                self.get_logger().info('Effort length back in sync with joint_names')
                self.effort_mismatch_reported = False
        elif msg.effort:
            if not self.effort_mismatch_reported:
                self.get_logger().warning(
                    f'Received {len(msg.effort)} efforts but expected {len(self.joint_names)}; discarding efforts'
                )
                self.effort_mismatch_reported = True

        self.get_logger().debug(
            f"Publishing joint state frame={msg.header.frame_id} positions={joint_state.position}")
        self.joint_state_pub.publish(joint_state)
        if self.mismatch_reported:
            self.get_logger().info('Resumed publishing joint states after size mismatch')
            self.mismatch_reported = False


def main(args=None):
    rclpy.init(args=args)
    node = IsaacStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
