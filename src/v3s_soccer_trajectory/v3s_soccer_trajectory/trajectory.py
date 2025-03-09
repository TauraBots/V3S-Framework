# trajectory.py (ou outro módulo onde está a classe base)
from abc import ABC, abstractmethod
from geometry_msgs.msg import PoseStamped, Twist
from v3s_soccer_interfaces.msg import FieldData

class Trajectory(ABC):
    """Classe base abstrata para algoritmos de planejamento de trajetória."""
    
    def __init__(self, plot_queue):
        self.max_linear_speed = 1000.0
        self.max_angular_speed = 50.0
        self.plot_queue = plot_queue
        self.current_waypoints = None
        self.current_field_data = None
        self.current_target_pose = None
        self.current_robot_index = 0

    def update_plot_data(self, waypoints, field_data, target_pose, robot_index):
        """
        Envia os dados de plotagem para a queue, para que o processo de plotagem atualize a janela.
        """
        self.plot_queue.put((waypoints, field_data, target_pose, robot_index))
    
    @abstractmethod
    def plan(self, field_data: FieldData, robot_index: int, target_pose: PoseStamped):
        """
        Planeja a trajetória e retorna uma lista de waypoints.
        """
        pass
    
    @abstractmethod
    def get_velocity_command(self, field_data: FieldData, robot_index: int, waypoints=None):
        """
        Calcula e retorna um objeto Twist com os comandos de velocidade.
        """
        pass
