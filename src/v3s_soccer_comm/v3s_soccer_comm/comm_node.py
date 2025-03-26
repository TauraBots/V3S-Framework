import socket
import struct
import json
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from google.protobuf.json_format import MessageToJson
from V3SProtoComm.core.comm.protocols import packet_pb2, wrapper_pb2

from V3SProtoComm.core.command import TeamCommand
from V3SProtoComm.core.comm.controls import ProtoControl

from v3s_soccer_interfaces.msg import FieldData as FieldDataMsg
from v3s_soccer_interfaces.msg import Ball as BallMsg
from v3s_soccer_interfaces.msg import Robot as RobotMsg


class CommNode(Node):
    def __init__(self):
        super().__init__('comm_node')

        # Escolha aqui o IP conforme se é simulador (224.0.0.1) ou visão (224.5.23.2)
        self.declare_parameter('receiver_ip', '224.5.23.2')    
        self.declare_parameter('receiver_port', 10015)         
        self.receiver_ip = self.get_parameter('receiver_ip').value
        self.receiver_port = self.get_parameter('receiver_port').value

        # Porta para enviar comandos ao simulador
        self.declare_parameter('simulator_port', 20011)
        simulator_port = self.get_parameter('simulator_port').value

        self.sock_recv = self._create_recv_socket()

        # Publica no tópico vision_data
        self.publisher_ = self.create_publisher(FieldDataMsg, 'vision_data', 10)

        # Escuta comandos de velocidade
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
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.receiver_ip, self.receiver_port))
        mreq = struct.pack("4sl", socket.inet_aton(self.receiver_ip), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.setblocking(False)
        return sock

    def timer_callback(self):
        """
        Lê o último pacote disponível do socket e decodifica:
          - Se self.receiver_ip == '224.0.0.1' => simulador (Environment)
          - Se self.receiver_ip == '224.5.23.2' => visão (SSL_WrapperPacket)
        """
        latest_data = None
        while True:
            try:
                data, addr = self.sock_recv.recvfrom(65535)
                latest_data = data
            except Exception:
                break

        if latest_data:
            field_data_msg = None

            # Decidir com base no IP:
            if self.receiver_ip == '224.0.0.1':
                # Parse como Simulador (Environment)
                env = packet_pb2.Environment()
                env.ParseFromString(latest_data)

                env_dict = json.loads(MessageToJson(env.frame))
                field_data_msg = self.convert_env_to_field_data_msg(env_dict)
                self.get_logger().debug("Decodificado como SIMULADOR (Environment)")

            elif self.receiver_ip == '224.5.23.2':
                # Parse como Visão SSL (SSL_WrapperPacket)
                ssl_msg = wrapper_pb2.SSL_WrapperPacket()
                ssl_msg.ParseFromString(latest_data)

                ssl_dict = json.loads(MessageToJson(ssl_msg))
                field_data_msg = self.convert_ssl_to_field_data_msg(ssl_dict)
                self.get_logger().debug("Decodificado como VISÃO (SSL_WrapperPacket)")

            # Se conseguiu montar o FieldDataMsg, publica
            if field_data_msg:
                self.publisher_.publish(field_data_msg)


    def convert_env_to_field_data_msg(self, data_dict):
        msg = FieldDataMsg()

        # BOLA
        ball_data = data_dict.get("ball", {})
        msg.ball.x = ball_data.get("x", 0.0)
        msg.ball.y = ball_data.get("y", 0.0)
        msg.ball.z = ball_data.get("z", 0.0)

        # ROBÔS AMARELOS
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

        # ROBÔS AZUIS
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
        msg = FieldDataMsg()

        detection = ssl_dict.get("detection", {})

        # BOLA (pega a 1ª da lista "balls")
        balls = detection.get("balls", [])
        if balls:
            b = balls[0]
            msg.ball.x = b.get("x", 0.0)
            msg.ball.y = b.get("y", 0.0)
            # se quiser z=0.0 pois SSL não usa

        # ROBÔS AMARELOS
        for r in detection.get("robotsYellow", []):
            robot = RobotMsg()
            robot.robot_id = r.get("robotId", 0)
            robot.x = r.get("x", 0.0)
            robot.y = r.get("y", 0.0)
            robot.orientation = r.get("orientation", 0.0)
            robot.vx = r.get("vx", 0.0)
            robot.vy = r.get("vy", 0.0)
            robot.vorientation = r.get("vorientation", 0.0)
            msg.robots_yellow.append(robot)

        # ROBÔS AZUIS
        for r in detection.get("robotsBlue", []):
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
