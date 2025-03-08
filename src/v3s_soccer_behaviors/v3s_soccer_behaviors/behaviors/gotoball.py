import math
from v3s_soccer_behaviors.behavior import Behavior

class GoToBall(Behavior):
    def __init__(self, offset_distance=0.3):
        """
        Inicializa o comportamento de ir à bola.
        
        Args:
            offset_distance: Distância a manter da bola (para não colidir)
        """
        self.offset_distance = offset_distance

    def execute(self, field_data, robot_index):
        """
        DEPRECATED: Mantido para compatibilidade.
        Use get_target_pose() em vez disso.
        """
        pass
        
    def get_target_pose(self, field_data, robot_index):
        """
        Determina a posição alvo para o robô com base na posição da bola.
        
        Args:
            field_data: Dados do campo incluindo posições dos robôs e da bola
            robot_index: Índice do robô para o qual calcular o alvo
            
        Returns:
            Tupla (x, y, theta) com as coordenadas e orientação alvo
        """
        ball = field_data.ball
        if not field_data.robots_blue or len(field_data.robots_blue) <= robot_index:
            return 0.0, 0.0, 0.0  
        

        ball_x = ball.x
        ball_y = ball.y
        
        robot = field_data.robots_blue[robot_index]
        robot_x = robot.x
        robot_y = robot.y
        
        dx = ball_x - robot_x
        dy = ball_y - robot_y
        distance = math.hypot(dx, dy)
        

        if distance > 0:
            dx /= distance
            dy /= distance
        
        if distance <= self.offset_distance:
            target_x = robot_x
            target_y = robot_y
        else:

            target_x = ball_x - (dx * self.offset_distance)
            target_y = ball_y - (dy * self.offset_distance)

        target_orientation = math.atan2(ball_y - target_y, ball_x - target_x)
        
        return target_x, target_y, target_orientation