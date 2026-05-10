#!/usr/bin/env python3


import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Forward speeds
        self.cruise_speed   = 0.6
        self.turn_speed     = 0.35
        self.search_speed   = 0.4

        # Offset thresholds
        self.large_offset   = 0.4
        self.centered_tol   = 0.05

        # P-control gain on steering
        self.steering_gain  = 1.6
        self.max_angular_z  = 1.5

        # Search behavior
        self.search_weave_rate = 0.5

        # State
        self.target_offset  = float('nan')
        self.target_visible = False
        self.start_time     = self.get_clock().now()

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32, '/target_offset',  self.offset_cb,  10)
        self.create_subscription(Bool,    '/target_visible', self.visible_cb, 10)

        # Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('control_node started — Ackermann perception-to-action.')

    def offset_cb(self, msg: Float32):
        self.target_offset = float(msg.data)

    def visible_cb(self, msg: Bool):
        self.target_visible = bool(msg.data)

    def control_loop(self):
        twist = Twist()

        if self.target_visible and not math.isnan(self.target_offset):
            offset = self.target_offset

            # Steer toward the box. Negative gain converts offset->angular.z
            # (offset>0 box on right -> angular.z<0 turn right)
            ang = -self.steering_gain * offset
            ang = max(-self.max_angular_z, min(self.max_angular_z, ang))

            if abs(offset) > self.large_offset:
                lin = self.turn_speed
                state = 'CHASE-TURN'
            else:
                lin = self.cruise_speed
                state = 'CHASE-FWD'

            twist.linear.x  = lin
            twist.angular.z = ang

            self.get_logger().info(
                f'{state}  offset={offset:+.2f}  lin={lin:.2f}  ang={ang:+.2f}',
                throttle_duration_sec=0.5)

        else:
            # SEARCH: drive forward + gentle weave (no spin in place)
            t_now = self.get_clock().now()
            elapsed = (t_now - self.start_time).nanoseconds * 1e-9
            weave = self.search_weave_rate * math.sin(0.4 * elapsed)

            twist.linear.x  = self.search_speed
            twist.angular.z = weave

            self.get_logger().info(
                f'SEARCH  weave={weave:+.2f}',
                throttle_duration_sec=1.0)

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
