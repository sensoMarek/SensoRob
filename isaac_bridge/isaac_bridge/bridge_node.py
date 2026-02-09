import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np

class IsaacMoveItBridge(Node):

    def __init__(self):
        super().__init__('isaac_moveit_bridge')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/sensorob_group_controller/follow_joint_trajectory',
            self.execute_callback)

        self.declare_parameter('output_topic', '/isaac/joint_commands')
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.publisher_ = self.create_publisher(Float64MultiArray, output_topic, 10)
        self.get_logger().info("Isaac-MoveIt bridge started.")
        self.rate = self.create_rate(100) # 100Hz

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        trajectory = goal_handle.request.trajectory
        if not trajectory.points:
            goal_handle.abort()
            self.get_logger().error("Trajectory has no points")
            return FollowJointTrajectory.Result()

        # Interpolate the trajectory
        joint_names = trajectory.joint_names
        trajectory_points = []
        for point in trajectory.points:
            trajectory_points.append(point.positions)

        # Full trajectory as a list of lists
        full_trajectory = self.interpolate_trajectory(trajectory_points, trajectory.points)

        start_time = self.get_clock().now()
        for i, point in enumerate(full_trajectory):
            if not rclpy.ok():
                break

            msg = Float64MultiArray()
            msg.data = point
            self.publisher_.publish(msg)
            if i == 0:
                self.get_logger().debug(f'Publishing first point: {msg.data}')
            elif i == len(full_trajectory) - 1:
                self.get_logger().debug(f'Publishing last point: {msg.data}')
            self.rate.sleep()

        goal_handle.succeed()
        self.get_logger().info('Goal succeeded!')
        result = FollowJointTrajectory.Result()
        return result

    def interpolate_trajectory(self, trajectory_points, time_from_start_list):
        # A simple linear interpolation
        full_trajectory = []
        if len(trajectory_points) < 2:
            return trajectory_points

        for i in range(len(trajectory_points) - 1):
            p1 = np.array(trajectory_points[i])
            p2 = np.array(trajectory_points[i+1])
            t1 = rclpy.duration.Duration.from_msg(time_from_start_list[i].time_from_start).nanoseconds / 1e9
            t2 = rclpy.duration.Duration.from_msg(time_from_start_list[i+1].time_from_start).nanoseconds / 1e9
            
            if t2 <= t1:
                continue
                
            num_steps = int((t2 - t1) * 100) # 100 Hz
            if num_steps == 0: continue

            for j in range(num_steps):
                alpha = float(j) / num_steps
                interpolated_point = (1 - alpha) * p1 + alpha * p2
                full_trajectory.append(interpolated_point.tolist())
        
        full_trajectory.append(trajectory_points[-1])
        return full_trajectory

def main(args=None):
    rclpy.init(args=args)
    node = IsaacMoveItBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
