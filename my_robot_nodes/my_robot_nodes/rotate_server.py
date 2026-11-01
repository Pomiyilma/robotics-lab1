import asyncio
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_robot_interfaces.action import Rotate


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RotateServer(Node):

    def __init__(self):
        super().__init__('rotate_server')

        self.declare_parameter('turn_speed', 0.5)
        self.turn_speed = float(
            self.get_parameter('turn_speed').value
        )

        self.current_yaw = None

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.action_server = ActionServer(
            self,
            Rotate,
            '/rotate',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Rotate action server is ready.')

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation

        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.current_yaw = math.atan2(sin_yaw, cos_yaw)

    def goal_callback(self, goal_request):
        angle = goal_request.angle

        if not math.isfinite(angle) or abs(angle) < 0.001:
            self.get_logger().warning(
                'Rejected goal: rotation angle must be non-zero.'
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancellation requested.')
        return CancelResponse.ACCEPT

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    async def execute_callback(self, goal_handle):
        requested_angle = float(goal_handle.request.angle)
        target_angle = abs(requested_angle)
        direction = 1.0 if requested_angle > 0 else -1.0

        result = Rotate.Result()
        feedback = Rotate.Feedback()

        wait_count = 0

        while self.current_yaw is None and wait_count < 100:
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                result.turned = 0.0
                return result

            await asyncio.sleep(0.05)
            wait_count += 1

        if self.current_yaw is None:
            self.get_logger().error('No /odom messages received.')
            self.stop_robot()
            goal_handle.abort()
            result.turned = 0.0
            return result

        previous_yaw = self.current_yaw
        turned = 0.0

        command = Twist()
        command.angular.z = direction * abs(self.turn_speed)

        while abs(turned) < target_angle:
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                result.turned = float(turned)
                return result

            if self.current_yaw is not None:
                yaw_change = normalize_angle(
                    self.current_yaw - previous_yaw
                )
                turned += yaw_change
                previous_yaw = self.current_yaw

            feedback.turned = float(turned)
            goal_handle.publish_feedback(feedback)

            self.cmd_pub.publish(command)
            await asyncio.sleep(0.05)

        self.stop_robot()
        goal_handle.succeed()

        result.turned = float(turned)

        self.get_logger().info(
            f'Rotation completed: {turned:.3f} radians.'
        )

        return result


def main(args=None):
    rclpy.init(args=args)

    node = RotateServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
