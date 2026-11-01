import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus

from geometry_msgs.msg import Twist
from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')

        self.declare_parameter('side_length', 0.5)
        self.side_length = float(
            self.get_parameter('side_length').value
        )

        self.drive_client = ActionClient(
            self, DriveDistance, '/drive_distance'
        )

        self.rotate_client = ActionClient(
            self, Rotate, '/rotate'
        )

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def drive_feedback(self, feedback_msg):
        travelled = feedback_msg.feedback.travelled
        self.get_logger().info(
            f'Driving: {travelled:.2f} m travelled'
        )

    def rotate_feedback(self, feedback_msg):
        turned = feedback_msg.feedback.turned
        self.get_logger().info(
            f'Turning: {turned:.2f} radians'
        )

    def send_drive_goal(self, side):
        goal = DriveDistance.Goal()
        goal.distance = self.side_length

        self.get_logger().info(
            f'Side {side}: driving {self.side_length:.2f} m'
        )

        future = self.drive_client.send_goal_async(
            goal,
            feedback_callback=self.drive_feedback
        )
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f'Side {side}: drive goal rejected.'
            )
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        response = result_future.result()

        if response is None:
            self.get_logger().error(
                f'Side {side}: no drive result received.'
            )
            return False

        result = response.result

        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or not result.reached
        ):
            self.get_logger().error(
                f'Side {side}: drive failed. '
                f'Travelled {result.travelled:.2f} m.'
            )
            return False

        self.get_logger().info(
            f'Side {side}: reached {result.travelled:.2f} m.'
        )
        return True

    def send_rotate_goal(self, side):
        goal = Rotate.Goal()
        goal.angle = 1.5708

        self.get_logger().info(
            f'Side {side}: rotating 90 degrees left.'
        )

        future = self.rotate_client.send_goal_async(
            goal,
            feedback_callback=self.rotate_feedback
        )
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f'Side {side}: rotation goal rejected.'
            )
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        response = result_future.result()

        if response is None:
            self.get_logger().error(
                f'Side {side}: no rotation result received.'
            )
            return False

        if response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'Side {side}: rotation failed.'
            )
            return False

        self.get_logger().info(
            f'Side {side}: rotation completed.'
        )
        return True

    def drive_square(self):
        self.get_logger().info(
            'Waiting for the robot action servers...'
        )

        self.drive_client.wait_for_server()
        self.rotate_client.wait_for_server()

        self.get_logger().info(
            'Both action servers are available. Starting square.'
        )

        for side in range(1, 5):
            if not self.send_drive_goal(side):
                self.stop_robot()
                self.get_logger().error(
                    f'Square stopped at side {side}.'
                )
                return

            if not self.send_rotate_goal(side):
                self.stop_robot()
                self.get_logger().error(
                    f'Square stopped after side {side}.'
                )
                return

        self.stop_robot()
        self.get_logger().info(
            'SUCCESS: The robot completed all four sides of the square!'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()

    try:
        node.drive_square()
    except KeyboardInterrupt:
        node.get_logger().info('Square drive interrupted.')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
