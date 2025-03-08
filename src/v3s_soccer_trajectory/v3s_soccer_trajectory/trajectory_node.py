#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from v3s_soccer_interfaces.msg import FieldData
from geometry_msgs.msg import Twist, PoseStamped

from v3s_soccer_trajectory.trajectories.direct_trajectory import DirectTrajectory

class TrajectoryNode(Node):
    """
    Nodo ROS responsável pelo planejamento e execução de trajetórias.
    Recebe alvos de comportamentos e gera comandos de velocidade para o robô.
    """
    
    def __init__(self):
        super().__init__('trajectory_node')
        

        self.declare_parameter('trajectory_algorithm', 'direct')
        self.declare_parameter('robot_index', 0)
        

        self.trajectory_algorithm = self.get_parameter('trajectory_algorithm').value
        self.robot_index = self.get_parameter('robot_index').value
        

        self.trajectories = {
            'direct': DirectTrajectory(),
        }
        

        if self.trajectory_algorithm in self.trajectories:
            self.trajectory = self.trajectories[self.trajectory_algorithm]
        else:
            self.get_logger().warning(
                f"Algoritmo '{self.trajectory_algorithm}' não encontrado. Usando 'direct' como fallback.")
            self.trajectory = self.trajectories['direct']
            

        self.field_data_sub = self.create_subscription(
            FieldData,
            'vision_data',
            self.field_data_callback,
            10)
            
        self.target_sub = self.create_subscription(
            PoseStamped,
            'target_pose',
            self.target_callback,
            10)
            

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
            

        self.current_field_data = None
        self.target_pose = None
        self.waypoints = []
        
        self.timer = self.create_timer(0.01, self.timer_callback)  
        
        self.get_logger().info(
            f"TrajectoryNode inicializado usando algoritmo '{self.trajectory_algorithm}'")
    
    def field_data_callback(self, msg):
        """Processa dados do campo recebidos."""
        self.current_field_data = msg
    
    def target_callback(self, msg):
        """Recebe um novo alvo para planejamento de trajetória."""
        self.target_pose = msg
        if self.current_field_data is not None:
            self.waypoints = self.trajectory.plan(
                self.current_field_data, 
                self.robot_index, 
                self.target_pose)
            self.get_logger().info(f"Nova trajetória planejada com {len(self.waypoints)} waypoints")
    
    def timer_callback(self):
        """Calcula e publica comandos de velocidade baseados na trajetória."""
        if self.current_field_data is None:
            return
            
        if not self.waypoints and self.target_pose is not None:
            self.waypoints = self.trajectory.plan(
                self.current_field_data, 
                self.robot_index, 
                self.target_pose)
        
        cmd_vel = self.trajectory.get_velocity_command(
            self.current_field_data,
            self.robot_index,
            self.waypoints)
            
        self.cmd_vel_pub.publish(cmd_vel)
    
    def change_trajectory_algorithm(self, algorithm_name):
        """
        Muda o algoritmo de trajetória em tempo de execução.
        """
        if algorithm_name in self.trajectories:
            self.trajectory_algorithm = algorithm_name
            self.trajectory = self.trajectories[algorithm_name]
            self.waypoints = []  
            self.get_logger().info(f"Algoritmo de trajetória alterado para '{algorithm_name}'")
            return True
        else:
            self.get_logger().error(f"Algoritmo '{algorithm_name}' não encontrado")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()