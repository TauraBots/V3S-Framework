import math
from geometry_msgs.msg import PoseStamped, Twist
from v3s_soccer_interfaces.msg import FieldData
from v3s_soccer_trajectory.trajectory import Trajectory

class DirectTrajectory(Trajectory):
    """Implementação simples de trajetória em linha reta direta ao alvo."""
    
    def __init__(self, k_linear=1.0, k_angular=3.0, stop_distance=0.05, angle_threshold=0.1):
        """
        Inicializa a trajetória direta.
        
        Args:
            k_linear: Ganho para controle de velocidade linear
            k_angular: Ganho para controle de velocidade angular
            stop_distance: Distância do alvo para considerar que chegou
            angle_threshold: Limiar de ângulo para ajuste de orientação
        """
        super().__init__()
        self.k_linear = k_linear
        self.k_angular = k_angular
        self.stop_distance = stop_distance
        self.angle_threshold = angle_threshold
        self.current_waypoints = []
    
    def plan(self, field_data: FieldData, robot_index: int, target_pose: PoseStamped):
        """
        Planeja uma trajetória direta entre o robô e o alvo.
        """
        # Para uma trajetória direta, só precisamos do ponto final
        self.current_waypoints = [target_pose]
        return self.current_waypoints
    
    def get_velocity_command(self, field_data: FieldData, robot_index: int, waypoints=None):
        """
        Calcula velocidades para mover o robô diretamente para o alvo.
        """
        if waypoints is not None:
            self.current_waypoints = waypoints
            
        if not self.current_waypoints:
            # Sem waypoints, parar o robô
            cmd_vel = Twist()
            return cmd_vel
            
        target_pose = self.current_waypoints[0]
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        
        # Obter posição atual do robô
        if not field_data.robots_blue or len(field_data.robots_blue) <= robot_index:
            cmd_vel = Twist()
            return cmd_vel
            
        robot = field_data.robots_blue[robot_index]
        robot_x = robot.x
        robot_y = robot.y
        robot_theta = robot.orientation
        
        # Calcular distância e ângulo para o alvo
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        
        # Calcular erro de ângulo
        angle_error = angle_to_target - robot_theta
        # Normalizar para [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # Criar comando de velocidade
        cmd_vel = Twist()
        
        if distance < self.stop_distance:
            # Chegou ao objetivo
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        else:
            # Prioridade ao ajuste de ângulo se o erro for grande
            if abs(angle_error) > self.angle_threshold:
                # Reduzir velocidade linear quando ajustando ângulo
                scaling_factor = max(0.3, 1.0 - (abs(angle_error) / math.pi))
                cmd_vel.linear.x = min(self.k_linear * distance * scaling_factor, self.max_linear_speed)
            else:
                # Velocidade linear normal
                cmd_vel.linear.x = min(self.k_linear * distance, self.max_linear_speed)
                
            # Velocidade angular proporcional ao erro de ângulo
            cmd_vel.angular.z = min(max(self.k_angular * angle_error, -self.max_angular_speed), self.max_angular_speed)
            
        return cmd_vel