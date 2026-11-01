import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed


class TurnAway(Node):

    def __init__(self):
        super().__init__('turn_away')

        # Parameters control the robot's movement.
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('max_turn_speed', 1.5)

        # The node starts disabled, as required by the exercise.
        self.enabled = False

        # Publisher: sends movement commands to the robot.
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # Publisher: reports the robot's current state.
        self.status_publisher = self.create_publisher(
            RobotStatus, '/robot_status', 10
        )

        # Subscriber: receives obstacle information.
        self.obstacle_subscription = self.create_subscription(
            ObstacleInfo,
            '/obstacle',
            self.obstacle_callback,
            10
        )

        # Service: enables or disables obstacle avoidance.
        self.enable_service = self.create_service(
            SetBool,
            '/turn_away_enable',
            self.enable_callback
        )

        # Service: changes the turning speed.
        self.speed_service = self.create_service(
            SetTurnSpeed,
            '/set_turn_speed',
            self.set_turn_speed_callback
        )

        self.get_logger().info(
            'turn_away started. It is currently DISABLED.'
        )

    def publish_zero_velocity(self):
        """Send a command that stops the robot."""
        command = Twist()
        self.cmd_vel_publisher.publish(command)

    def publish_status(self, state, obstacle_distance):
        """Publish the robot's current state."""
        status = RobotStatus()
        status.state = state
        status.obstacle_distance = float(obstacle_distance)
        self.status_publisher.publish(status)

    def obstacle_callback(self, msg):
        """React whenever new obstacle information arrives."""

        if not self.enabled:
            self.publish_status('stopped', msg.distance)
            return

        command = Twist()

        if not msg.too_close:
            # No obstacle is too close: drive forward.
            forward_speed = float(
                self.get_parameter('forward_speed').value
            )
            command.linear.x = forward_speed
            command.angular.z = 0.0
            state = 'forward'

        else:
            # An obstacle is too close: turn on the spot.
            turn_speed = float(
                self.get_parameter('turn_speed').value
            )

            command.linear.x = 0.0

            if msg.angle > 0.0:
                # Obstacle is on the left: turn right.
                command.angular.z = -turn_speed
            else:
                # Obstacle is on the right or straight ahead: turn left.
                command.angular.z = turn_speed

            state = 'turning'

        self.cmd_vel_publisher.publish(command)
        self.publish_status(state, msg.distance)

    def enable_callback(self, request, response):
        """Switch obstacle avoidance on or off."""

        if request.data:
            self.enabled = True
            response.success = True
            response.message = 'Obstacle avoidance enabled.'
            self.get_logger().info(response.message)

        else:
            was_enabled = self.enabled
            self.enabled = False

            # Send one zero velocity when switching off.
            if was_enabled:
                self.publish_zero_velocity()

            response.success = True
            response.message = 'Obstacle avoidance disabled.'
            self.get_logger().info(response.message)

        return response

    def set_turn_speed_callback(self, request, response):
        """Accept only valid turning speeds."""

        requested_speed = float(request.turn_speed)
        max_speed = float(
            self.get_parameter('max_turn_speed').value
        )

        if not math.isfinite(requested_speed) or requested_speed <= 0.0:
            response.success = False
            response.message = 'Turn speed must be greater than zero.'

        elif requested_speed > max_speed:
            response.success = False
            response.message = (
                f'Turn speed exceeds the maximum of {max_speed:.2f} rad/s.'
            )

        else:
            self.set_parameters([
                Parameter(
                    'turn_speed',
                    Parameter.Type.DOUBLE,
                    requested_speed
                )
            ])

            response.success = True
            response.message = (
                f'Turn speed set to {requested_speed:.2f} rad/s.'
            )

            self.get_logger().info(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)

    node = TurnAway()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
