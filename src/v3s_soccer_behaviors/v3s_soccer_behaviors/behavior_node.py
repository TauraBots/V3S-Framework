#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from v3s_soccer_interfaces.msg import FieldData as FieldDataMsg
from geometry_msgs.msg import PoseStamped
from v3s_soccer_behaviors.behaviors.gotoball import GoToBall

class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_node')
        self.subscription = self.create_subscription(
            FieldDataMsg,
            'vision_data',
            self.vision_callback,
            10)

        self.publisher_ = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.behavior = GoToBall()
        self.robot_index = 0
        self.get_logger().info("BehaviorNode inicializado - publicando alvos para o sistema de trajetória.")

    def vision_callback(self, msg: FieldDataMsg):
        
        target_x, target_y, target_orientation = self.behavior.get_target_pose(msg, self.robot_index)
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = "map"
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.orientation.z = target_orientation
        

        self.publisher_.publish(target_pose)
        self.get_logger().debug(
            f"Alvo publicado: x={target_x:.2f}, y={target_y:.2f}, theta={target_orientation:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()