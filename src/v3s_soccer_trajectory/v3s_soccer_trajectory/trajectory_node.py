#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from v3s_soccer_interfaces.srv import SetTrajectory
from .straight_line_planner import StraightLinePlanner

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        self.declare_parameter('default_mode', 'straight')
        self.current_mode = self.get_parameter('default_mode').value.lower().strip()
        self.planner = StraightLinePlanner()
        self.srv = self.create_service(SetTrajectory, 'set_trajectory_mode', self.set_trajectory_callback)
        self.get_logger().info(f"TrajectoryNode inicializado com modo '{self.current_mode}'.")

    def set_trajectory_callback(self, request, response):
        mode = request.mode.lower().strip()
        if mode == "straight":
            self.planner = StraightLinePlanner()
            self.current_mode = "straight"
            response.success = True
            response.message = "Modo de trajetória alterado para 'straight'."
        else:
            response.success = False
            response.message = f"Modo '{mode}' desconhecido. Use 'straight' ou 'astar'."
        
        self.get_logger().info(f"SetTrajectory request: {mode} | Resposta: {response.message}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
