from abc import ABC, abstractmethod
from geometry_msgs.msg import PoseStamped, Twist
from v3s_soccer_interfaces.msg import FieldData
import matplotlib.pyplot as plt
import matplotlib.patches as patches

FIELD_WIDTH = 1.3
FIELD_LENGTH = 1.5

GOAL_WIDTH = 0.4
GOAL_DEPTH = 0.1

GOAL_AREA_WIDTH = 0.7
GOAL_AREA_DEPTH = 0.15

ROBOT_SIZE = 0.075
WHEEL_RADIUS = 0.025
BALL_RADIUS = 0.02135

class Trajectory(ABC):
    """Classe base abstrata para algoritmos de planejamento de trajetória."""
    
    def __init__(self):
        """Inicializa a classe base de trajetória."""
        self.max_linear_speed = 1000.0
        self.max_angular_speed = 50.0
        
        self.fig, self.ax = plt.subplots()
        self.draw_field()
        plt.ion() 
        plt.show()
    
    def draw_field(self):
        """Desenha o campo de futebol com as dimensões e elementos fixos."""
        self.ax.clear()
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("Campo de Robô Soccer")
        
        half_length = FIELD_LENGTH / 2.0
        half_width = FIELD_WIDTH / 2.0
        
        self.ax.set_xlim(-half_length - 0.3, half_length + 0.3)
        self.ax.set_ylim(-half_width - 0.3, half_width + 0.3)
        
        field_rect = patches.Rectangle((-half_length, -half_width),
                                       FIELD_LENGTH, FIELD_WIDTH,
                                       edgecolor='black', facecolor='none', lw=2)
        self.ax.add_patch(field_rect)
        
        self.ax.plot([0, 0], [-half_width, half_width], 'k--', lw=1)
        center_circle = patches.Circle((0, 0), 0.1, edgecolor='black', facecolor='none', lw=1)
        self.ax.add_patch(center_circle)
        
        goal_right = patches.Rectangle((half_length, -GOAL_WIDTH/2),
                                       GOAL_DEPTH, GOAL_WIDTH,
                                       edgecolor='red', facecolor='none', lw=2)
        self.ax.add_patch(goal_right)
        goal_area_right = patches.Rectangle((half_length - GOAL_AREA_DEPTH, -GOAL_AREA_WIDTH/2),
                                            GOAL_AREA_DEPTH, GOAL_AREA_WIDTH,
                                            edgecolor='red', facecolor='none', lw=1)
        self.ax.add_patch(goal_area_right)
        
        goal_left = patches.Rectangle((-half_length - GOAL_DEPTH, -GOAL_WIDTH/2),
                                      GOAL_DEPTH, GOAL_WIDTH,
                                      edgecolor='blue', facecolor='none', lw=2)
        self.ax.add_patch(goal_left)
        goal_area_left = patches.Rectangle((-half_length, -GOAL_AREA_WIDTH/2),
                                           GOAL_AREA_DEPTH, GOAL_AREA_WIDTH,
                                           edgecolor='blue', facecolor='none', lw=1)
        self.ax.add_patch(goal_area_left)

    def plot_trajectory(self, waypoints, field_data=None, target_pose=None, robot_index=0):
        """
        Atualiza o plot com os waypoints, a posição atual do robô e o alvo.
        
        Args:
            waypoints (list[PoseStamped]): Lista de waypoints formando a trajetória.
            field_data (FieldData): Dados do campo com a posição dos robôs.
            target_pose (PoseStamped): Posição alvo.
            robot_index (int): Índice do robô a ser exibido.
        """
        self.draw_field()

        if waypoints:
            x_vals = [wp.pose.position.x for wp in waypoints]
            y_vals = [wp.pose.position.y for wp in waypoints]
            self.ax.plot(x_vals, y_vals, 'bo-', label="Trajetória")
        
        if target_pose:
            tx = target_pose.pose.position.x
            ty = target_pose.pose.position.y
            self.ax.plot(tx, ty, 'rx', markersize=10, label="Alvo")
        
        if field_data and hasattr(field_data, 'robots_blue') and len(field_data.robots_blue) > robot_index:
            robot = field_data.robots_blue[robot_index]
            self.ax.plot(robot.x, robot.y, 'go', markersize=8, label="Robô")
            
        
        self.ax.legend()
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    @abstractmethod
    def plan(self, field_data: FieldData, robot_index: int, target_pose: PoseStamped):
        """
        Planeja uma trajetória com base nos dados de campo e no alvo.
        
        Args:
            field_data: Dados do campo, incluindo posições dos robôs e da bola.
            robot_index: Índice do robô para o qual planejar a trajetória.
            target_pose: Pose alvo para o robô.
            
        Returns:
            Uma lista de waypoints (PoseStamped) que formam a trajetória.
        """
        self.plot_trajectory(self.current_waypoints, field_data, target_pose, robot_index)
        pass
    
    @abstractmethod
    def get_velocity_command(self, field_data: FieldData, robot_index: int, waypoints=None):
        """
        Calcula comandos de velocidade para seguir a trajetória planejada.
        
        Args:
            field_data: Dados atualizados do campo, incluindo posições dos robôs e da bola.
            robot_index: Índice do robô para o qual calcular os comandos.
            waypoints: Lista de waypoints a seguir (opcional).
            
        Returns:
            Um objeto Twist com os comandos de velocidade linear e angular.
        """
        pass
