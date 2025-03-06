import socket
import struct
import math
import json
import rclpy
from rclpy.node import Node

from V3SProtoComm.core.comm.protocols import packet_pb2  
from google.protobuf.json_format import MessageToJson
from v3s_soccer_interfaces.msg import FieldData as FieldDataMsg, Ball as BallMsg, Robot as RobotMsg

class CommNode(Node):
    def __init__(self):
        super().__init__('comm_node')
        self.declare_parameter('receiver_ip', '224.0.0.1')
        self.declare_parameter('receiver_port', 10002)
        self.receiver_ip = self.get_parameter('receiver_ip').value
        self.receiver_port = self.get_parameter('receiver_port').value

        self.sock = self._create_socket()

        self.publisher_ = self.create_publisher(FieldDataMsg, 'vision_data', 120)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f"CommNode inicializado. Escutando em {self.receiver_ip}:{self.receiver_port}")

    def _create_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.receiver_ip, self.receiver_port))
        mreq = struct.pack("4sl", socket.inet_aton(self.receiver_ip), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.setblocking(False)
        return sock

    def timer_callback(self):
        try:
            latest_data = None
            while True:
                data, addr = self.sock.recvfrom(65535)
                latest_data = data
        except Exception:
            pass

        if latest_data:
            try:
                env = packet_pb2.Environment()
                env.ParseFromString(latest_data)
                json_str = MessageToJson(env.frame)
                data_dict = json.loads(json_str)
                field_data_msg = self.convert_to_field_data_msg(data_dict)
                self.publisher_.publish(field_data_msg)
            except Exception as e:
                self.get_logger().error(f"Erro ao decodificar ou converter os dados: {e}")

    def convert_to_field_data_msg(self, data_dict):
        """Converte o dicionário obtido a partir do Protobuf para a mensagem FieldDataMsg."""
        msg = FieldDataMsg()
        
        ball_data = data_dict.get("ball", {})
        ball = BallMsg()
        ball.x = ball_data.get("x", 0.0)
        ball.y = ball_data.get("y", 0.0)
        ball.z = ball_data.get("z", 0.0)
        msg.ball = ball

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

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
