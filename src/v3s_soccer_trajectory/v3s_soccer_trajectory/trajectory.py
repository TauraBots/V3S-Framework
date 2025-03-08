from abc import ABC, abstractmethod
from geometry_msgs.msg import PoseStamped, Twist
from v3s_soccer_interfaces.msg import FieldData

class Trajectory(ABC):
    """Classe base abstrata para algoritmos de planejamento de trajetória."""
    
    def __init__(self):
        """Inicializa a classe base de trajetória."""
        # Configurações comuns para todos os planejadores de trajetória
        self.max_linear_speed = 1.0  # m/s
        self.max_angular_speed = 2.0  # rad/s
        
    @abstractmethod
    def plan(self, field_data: FieldData, robot_index: int, target_pose: PoseStamped):
        """
        Planeja uma trajetória com base nos dados de campo e no alvo.
        
        Args:
            field_data: Dados do campo incluindo posições dos robôs e da bola
            robot_index: Índice do robô para o qual planejar a trajetória
            target_pose: Pose alvo para o robô
            
        Returns:
            Uma lista de waypoints (PoseStamped) que formam a trajetória
        """
        pass
    
    @abstractmethod
    def get_velocity_command(self, field_data: FieldData, robot_index: int, waypoints=None):
        """
        Calcula comandos de velocidade para seguir a trajetória planejada.
        
        Args:
            field_data: Dados atualizados do campo incluindo posições dos robôs e da bola
            robot_index: Índice do robô para o qual calcular os comandos
            waypoints: Lista de waypoints a seguir (opcional)
            
        Returns:
            Um objeto Twist com os comandos de velocidade linear e angular
        """
        pass