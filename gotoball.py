import time
import logging
import math
from V3SProtoComm.core.data import FieldData
from V3SProtoComm.core.comm.vision import ProtoVisionThread
from V3SProtoComm.core.comm.controls import ProtoControl
from V3SProtoComm.core.command import TeamCommand

def gotoball():
    logging.basicConfig(level=logging.INFO)
    print("\n=== Gotoball: Seguindo a bola com curva enquanto se move ===")
    
    # Inicializa a estrutura de dados que será atualizada pela thread de visão
    field_data = FieldData()
    
    # Inicia a thread de visão (team_color_yellow=False indica que os dados do time são os robôs azuis)
    vision_thread = ProtoVisionThread(team_color_yellow=False, field_data=field_data)
    vision_thread.start()
    print("Thread de visão iniciada.")
    
    # Inicializa o controle para enviar comandos de velocidade ao robô
    team_command = TeamCommand()
    control = ProtoControl(team_color_yellow=False, team_command=team_command, control_port=20011)
    
    # Define o índice do robô a ser controlado (0 para robô 0, 1 para robô 1, etc.)
    desired_robot_index = 0
    
    # Parâmetros do controlador
    k_turn = 10.0         # Ganho proporcional para rotação
    k_forward = 50.0      # Ganho proporcional para avanço
    stop_distance = 0.2   # Distância mínima para considerar que o robô "domina" a bola
    
    try:
        while True:
            ball = field_data.ball
            if field_data.robots and len(field_data.robots) > desired_robot_index:
                selected_robot = field_data.robots[desired_robot_index]
            else:
                selected_robot = None
            
            if ball is not None and selected_robot is not None:
                # Dados do robô
                robot_x = selected_robot.position.x
                robot_y = selected_robot.position.y
                robot_theta = selected_robot.position.theta
                
                # Dados da bola
                ball_x = ball.position.x
                ball_y = ball.position.y
                
                # Calcula o vetor do robô até a bola
                dx = ball_x - robot_x
                dy = ball_y - robot_y
                distance = math.hypot(dx, dy)
                
                # Calcula o ângulo desejado para se dirigir à bola
                angle_to_ball = math.atan2(dy, dx)
                
                # Calcula o erro angular entre a orientação atual do robô e o ângulo para a bola
                angle_error = angle_to_ball - robot_theta
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normaliza para [-pi, pi]
                
                if distance < stop_distance:
                    # Se estiver muito perto, para o robô
                    forward_speed = 0
                    turn_speed = 0
                else:
                    # Define a velocidade linear com base na distância
                    forward_speed = k_forward * distance
                    # Se o erro angular for grande, reduz a velocidade de avanço para permitir um giro mais suave.
                    # Por exemplo, se o erro for 90 graus (pi/2 rad), reduzimos o avanço a 50% (mínimo).
                    max_error = math.pi / 2  # 90 graus
                    scaling_factor = max(0.5, 1 - (abs(angle_error) / max_error))
                    forward_speed *= scaling_factor
                    # A velocidade de rotação é proporcional ao erro angular
                    turn_speed = k_turn * angle_error
                
                # Modelo diferencial: converte velocidades linear e angular para velocidades das rodas
                left_speed = forward_speed - turn_speed
                right_speed = forward_speed + turn_speed
                
                # Atualiza os comandos do robô selecionado
                team_command.commands[desired_robot_index].left_speed = left_speed
                team_command.commands[desired_robot_index].right_speed = right_speed
                
                # Envia os comandos para o robô
                control.update()
                
                # Exibe os dados para debug
                print("=== Dados de Controle ===")
                print("Bola -> x: {:.2f}, y: {:.2f}, Distância: {:.2f}".format(ball_x, ball_y, distance))
                print("Robô (índice {}) -> x: {:.2f}, y: {:.2f}, θ: {:.2f}".format(
                    desired_robot_index, robot_x, robot_y, robot_theta))
                print("Ângulo desejado: {:.2f} rad | Erro angular: {:.2f} rad".format(angle_to_ball, angle_error))
                print("Comando -> Esquerda: {:.2f}, Direita: {:.2f}".format(left_speed, right_speed))
            else:
                print("Dados incompletos: bola ou robô não disponível.")
            
            print("-" * 40)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        logging.info("Encerrando gotoball.")

if __name__ == "__main__":
    gotoball()
