import math
from v3s_soccer_behaviors.behavior import Behavior

class GoToBall(Behavior):
    def __init__(self, k_turn=10.0, k_forward=50.0, stop_distance=0.2, angle_threshold=0.1):
        self.k_turn = k_turn
        self.k_forward = k_forward
        self.stop_distance = stop_distance
        self.angle_threshold = angle_threshold

    def execute(self, field_data, robot_index):
        """
        Processa a mensagem FieldData e retorna uma tupla (left_speed, right_speed)
        para o robô identificado por robot_index.
        """
        ball = field_data.ball
        if not field_data.robots_blue or len(field_data.robots_blue) <= robot_index:
            return 0.0, 0.0  
        
        robot = field_data.robots_blue[robot_index]
        robot_x = robot.x
        robot_y = robot.y
        robot_theta = robot.orientation

        ball_x = ball.x
        ball_y = ball.y

        dx = ball_x - robot_x
        dy = ball_y - robot_y
        distance = math.hypot(dx, dy)

        angle_to_ball = math.atan2(dy, dx)
        angle_error = angle_to_ball - robot_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if distance < self.stop_distance:
            forward_speed = 0.0
            turn_speed = 0.0
        else:
            if abs(angle_error) > self.angle_threshold:
                scaling_factor = max(0.5, 1 - (abs(angle_error) / (math.pi / 2)))
                forward_speed = self.k_forward * distance * scaling_factor
            else:
                forward_speed = self.k_forward * distance
            turn_speed = self.k_turn * angle_error

        left_speed = forward_speed - turn_speed
        right_speed = forward_speed + turn_speed

        return left_speed, right_speed
