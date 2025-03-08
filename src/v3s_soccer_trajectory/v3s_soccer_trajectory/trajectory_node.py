#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from v3s_soccer_interfaces.msg import FieldData
from geometry_msgs.msg import Twist, PoseStamped

# Importar implementações de trajetória
from v3s_soccer_trajectory.trajectories.direct_trajectory import DirectTrajectory
from v3s_soccer_trajectory.trajectories.astar_trajectory import AStarTrajectory
from v3s_soccer_trajectory.trajectories.rrt_trajectory import RRTTrajectory
from v3s_soccer_trajectory.trajectories.vector_field_trajectory import VectorFieldTrajectory

class TrajectoryNode(Node):
    """
    Nodo ROS responsável pelo planejamento e execução de trajetórias.
    Recebe alvos de comportamentos e gera comandos de velocidade para o robô.
    """
    
    def __init__(self):
        super().__init__('trajectory_node')
        
        # Parâmetros de configuração
        self.declare_parameter('trajectory_algorithm', 'direct')
        self.declare_parameter('robot_index', 0)
        
        # Obter valores dos parâmetros
        self.trajectory_algorithm = self.get_parameter('trajectory_algorithm').value
        self.robot_index = self.get_parameter('robot_index').value
        
        # Inicializar trajetórias disponíveis
        self.trajectories = {
            'direct': DirectTrajectory(),
            # Comentado até que sejam implementados
            # 'astar': AStarTrajectory(),
            # 'rrt': RRTTrajectory(),
            # 'vector_field': VectorFieldTrajectory()
        }
        
        # Selecionar o algoritmo apropriado
        if self.trajectory_algorithm in self.trajectories:
            self.trajectory = self.trajectories[self.trajectory_algorithm]
        else:
            self.get_logger().warning(
                f"Algoritmo '{self.trajectory_algorithm}' não encontrado. Usando 'direct' como fallback.")
            self.trajectory = self.trajectories['direct']
            
        # Criar subscriptions
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
            
        # Criar publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
            
        # Estado interno
        self.current_field_data = None
        self.target_pose = None
        self.waypoints = []
        
        # Timer para calcular e publicar comandos de velocidade
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
        self.get_logger().info(
            f"TrajectoryNode inicializado usando algoritmo '{self.trajectory_algorithm}'")
    
    def field_data_callback(self, msg):
        """Processa dados do campo recebidos."""
        self.current_field_data = msg
    
    def target_callback(self, msg):
        """Recebe um novo alvo para planejamento de trajetória."""
        self.target_pose = msg
        # Planejar trajetória quando receber um novo alvo
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
            # Se não temos waypoints mas temos um alvo, planejar a trajetória
            self.waypoints = self.trajectory.plan(
                self.current_field_data, 
                self.robot_index, 
                self.target_pose)
        
        # Obter comando de velocidade do planejador de trajetória
        cmd_vel = self.trajectory.get_velocity_command(
            self.current_field_data,
            self.robot_index,
            self.waypoints)
            
        # Publicar comando
        self.cmd_vel_pub.publish(cmd_vel)
    
    def change_trajectory_algorithm(self, algorithm_name):
        """
        Muda o algoritmo de trajetória em tempo de execução.
        """
        if algorithm_name in self.trajectories:
            self.trajectory_algorithm = algorithm_name
            self.trajectory = self.trajectories[algorithm_name]
            self.waypoints = []  # Limpar waypoints atuais
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