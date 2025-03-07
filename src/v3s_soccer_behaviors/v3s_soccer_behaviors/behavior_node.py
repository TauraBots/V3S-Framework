#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from v3s_soccer_interfaces.msg import FieldData as FieldDataMsg
from geometry_msgs.msg import Twist
from v3s_soccer_behaviors.behaviors.gotoball import GoToBall

class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_node')
        self.subscription = self.create_subscription(
            FieldDataMsg,
            'vision_data',
            self.vision_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 120)
        self.declare_parameter('planner_mode', 'straight')
        planner_mode = self.get_parameter('planner_mode').value
        self.behavior = GoToBall(planner_mode=planner_mode)
        self.robot_index = 0
        self.get_logger().info(f"BehaviorNode inicializado com planner_mode='{planner_mode}'.")

    def vision_callback(self, msg: FieldDataMsg):
        left_speed, right_speed = self.behavior.execute(msg, self.robot_index)
        wheel_separation = 0.5
        linear_vel = (left_speed + right_speed) / 2.0
        angular_vel = (right_speed - left_speed) / wheel_separation

        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.publisher_.publish(twist)
        self.get_logger().info(
            f"CmdVel publicado: linear = {linear_vel:.2f} m/s, angular = {angular_vel:.2f} rad/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
