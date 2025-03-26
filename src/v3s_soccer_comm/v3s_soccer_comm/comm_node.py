import socket
import struct
import json
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

# PROTOBUF
from google.protobuf.json_format import MessageToJson
from V3SProtoComm.core.comm.protocols import packet_pb2, wrapper_pb2

# Classes de comandos (seu controle para o simulador)
from V3SProtoComm.core.command import TeamCommand
from V3SProtoComm.core.comm.controls import ProtoControl

# Mensagens ROS2 personalizadas
from v3s_soccer_interfaces.msg import FieldData as FieldDataMsg
from v3s_soccer_interfaces.msg import Ball as BallMsg
from v3s_soccer_interfaces.msg import Robot as RobotMsg

class CommNode(Node):
    def __init__(self):
        super().__init__('comm_node')

        self.declare_parameter('receiver_ip', '224.0.0.1')    # Simulador, por padrão
        self.declare_parameter('receiver_port', 10002)        # Porta do simulador
        self.receiver_ip = self.get_parameter('receiver_ip').value
        self.receiver_port = self.get_parameter('receiver_port').value

        self.declare_parameter('simulator_port', 20011)
        simulator_port = self.get_parameter('simulator_port').value

        self.sock_recv = self._create_recv_socket()

        self.publisher_ = self.create_publisher(FieldDataMsg, 'vision_data', 10)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10
        )

        self.timer = self.create_timer(1/120, self.timer_callback)

        self.get_logger().info(f"CommNode inicializado. Recebendo de {self.receiver_ip}:{self.receiver_port}")
        self.get_logger().info(f"Enviando para simulador na porta {simulator_port}")

        self.team_command = TeamCommand()
        self.control = ProtoControl(team_color_yellow=False, team_command=self.team_command, control_port=simulator_port)

        self.robot_index = 0

    def _create_recv_socket(self):
        """
        Cria e configura um socket multicast para receber pacotes
        de (self.receiver_ip, self.receiver_port).
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.receiver_ip, self.receiver_port))

        mreq = struct.pack("4sl", socket.inet_aton(self.receiver_ip), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.setblocking(False)  
        return sock

    def timer_callback(self):

        latest_data = None
        while True:
            try:
                data, addr = self.sock_recv.recvfrom(65535)
                latest_data = data
            except Exception:
                break  

        if latest_data:
            field_data_msg = None
            try:
                env = packet_pb2.Environment()
                env.ParseFromString(latest_data)

                env_dict = json.loads(MessageToJson(env.frame))

                field_data_msg = self.convert_env_to_field_data_msg(env_dict)

                self.get_logger().debug("Pacote decodificado como SIMULADOR (Environment)")

            except Exception as e_env:
                try:
                    ssl_msg = wrapper_pb2.SSL_WrapperPacket()
                    ssl_msg.ParseFromString(latest_data)

                    ssl_dict = json.loads(MessageToJson(ssl_msg))

                    field_data_msg = self.convert_ssl_to_field_data_msg(ssl_dict)

                    self.get_logger().debug("Pacote decodificado como VISÃO (SSL_WrapperPacket)")

                except Exception as e_ssl:
                    self.get_logger().error(f"Falha ao decodificar como Env ou SSL: {e_env} / {e_ssl}")
                    field_data_msg = None

            if field_data_msg:
                self.publisher_.publish(field_data_msg)


    def convert_env_to_field_data_msg(self, data_dict):
        """
        Converte o dicionário proveniente de Environment().frame
        (que tem 'ball', 'robotsYellow', 'robotsBlue' etc.)
        em FieldDataMsg para publicação.
        """
        msg = FieldDataMsg()

        ball_data = data_dict.get("ball", {})
        ball_msg = BallMsg()
        ball_msg.x = ball_data.get("x", 0.0)
        ball_msg.y = ball_data.get("y", 0.0)
        ball_msg.z = ball_data.get("z", 0.0)
        msg.ball = ball_msg

        for r in data_dict.get("robotsYellow", []):
            robot = RobotMsg()
            robot.robot_id = r.get("robotId", 0)
            robot.x = r.get("x", 0.0)
            robot.y = r.get("y", 0.0)
            robot.orientation = r.get("orientation", 0.0)
            robot.vx = r.get("vx", 0.0)
            robot.vy = r.get("vy", 0.0)
            robot.vorientation = r.get("vorientation", 0.0)
            msg.robots_yellow.append(robot)

        for r in data_dict.get("robotsBlue", []):
            robot = RobotMsg()
            robot.robot_id = r.get("robotId", 0)
            robot.x = r.get("x", 0.0)
            robot.y = r.get("y", 0.0)
            robot.orientation = r.get("orientation", 0.0)
            robot.vx = r.get("vx", 0.0)
            robot.vy = r.get("vy", 0.0)
            robot.vorientation = r.get("vorientation", 0.0)
            msg.robots_blue.append(robot)

        return msg

    def convert_ssl_to_field_data_msg(self, ssl_dict):
        """
        Converte o dicionário proveniente de SSL_WrapperPacket
        (normalmente campos em ssl_dict['detection'])
        em FieldDataMsg para publicação.
        """
        msg = FieldDataMsg()

        detection = ssl_dict.get("detection", {})

        balls = detection.get("balls", [])
        if len(balls) > 0:
            b = balls[0]
            ball_msg = BallMsg()
            ball_msg.x = b.get("x", 0.0)
            ball_msg.y = b.get("y", 0.0)
            msg.ball = ball_msg

        robots_yellow = detection.get("robotsYellow", [])
        for r in robots_yellow:
            robot = RobotMsg()
            robot.robot_id = r.get("robotId", 0)
            robot.x = r.get("x", 0.0)
            robot.y = r.get("y", 0.0)
            robot.orientation = r.get("orientation", 0.0)
            robot.vx = r.get("vx", 0.0)
            robot.vy = r.get("vy", 0.0)
            robot.vorientation = r.get("vorientation", 0.0)
            msg.robots_yellow.append(robot)

        robots_blue = detection.get("robotsBlue", [])
        for r in robots_blue:
            robot = RobotMsg()
            robot.robot_id = r.get("robotId", 0)
            robot.x = r.get("x", 0.0)
            robot.y = r.get("y", 0.0)
            robot.orientation = r.get("orientation", 0.0)
            robot.vx = r.get("vx", 0.0)
            robot.vy = r.get("vy", 0.0)
            robot.vorientation = r.get("vorientation", 0.0)
            msg.robots_blue.append(robot)

        return msg


    def cmd_callback(self, msg: Twist):
        """
        Exemplo de envio de velocidade para o simulador.
        """
        wheel_separation = 0.5
        left_speed = msg.linear.x - (msg.angular.z * wheel_separation / 2.0)
        right_speed = msg.linear.x + (msg.angular.z * wheel_separation / 2.0)

        self.team_command.commands[self.robot_index].left_speed = left_speed 
        self.team_command.commands[self.robot_index].right_speed = right_speed 

        # Manda pro ProtoControl (que deve estar ligado ao simulador)
        self.control.update()

        self.get_logger().info(f"Comando enviado: wheel_left={left_speed:.2f}, wheel_right={right_speed:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
