import time
import logging
import math

from V3SProtoComm.core.data import FieldData
from V3SProtoComm.core.comm.vision import ProtoVisionThread
from V3SProtoComm.core.comm.controls import ProtoControl
from V3SProtoComm.core.command import TeamCommand

def main():
    logging.basicConfig(level=logging.INFO)
    print("=== Teste da Biblioteca V3SProtoComm ===")
    
    field_data = FieldData()
    
    vision_thread = ProtoVisionThread(team_color_yellow=False, field_data=field_data)
    vision_thread.start()
    print("Thread de visão iniciada.")
    
    team_command = TeamCommand()
    control = ProtoControl(team_color_yellow=False, team_command=team_command, control_port=20011)
    
    desired_robot_index = 0
    
    k_turn = 10.0         # Ganho proporcional para rotação
    k_forward = 10.0      # Ganho proporcional para avanço
    stop_distance = 0.2   # Distância mínima para considerar que o robô "domina" a bola
    angle_threshold = 0.1 # Erro angular mínimo para avanço (em radianos)
    
    try:
        while True:
            # Atualiza os dados do campo (bola e robôs) que já são atualizados pela thread de visão
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
                
                # Calcula o vetor do robô até a bola e a distância
                dx = ball_x - robot_x
                dy = ball_y - robot_y
                distance = math.hypot(dx, dy)
                
                # Calcula o ângulo desejado para se dirigir à bola
                angle_to_ball = math.atan2(dy, dx)
                
                # Calcula o erro angular entre a orientação atual do robô e o ângulo para a bola
                angle_error = angle_to_ball - robot_theta
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normaliza para [-pi, pi]
                
                # Estratégia de controle:
                # Se o robô estiver muito próximo da bola, para o movimento.
                # Se o erro angular for grande, prioriza a correção da orientação (sem avanço).
                if distance < stop_distance:
                    forward_speed = 0
                    turn_speed = 0
                else:
                    if abs(angle_error) > angle_threshold:
                        forward_speed = 0
                    else:
                        forward_speed = k_forward * distance
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
        logging.info("Encerrando teste da V3SProtoComm.")

if __name__ == "__main__":
    main()
