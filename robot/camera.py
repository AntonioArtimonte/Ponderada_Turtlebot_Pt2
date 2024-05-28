import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import base64
import time

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Erro: Não foi possível acessar a webcam.")
            return
        self.cap.set(cv2.CAP_PROP_FPS, 30)  # Configura a câmera para 30 FPS, se suportado
        timer_period = 0.0167  # aproximadamente 0.0167 segundos (60 FPS)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            timestamp = time.time()
            msg = String()
            msg.data = f"{timestamp}|{jpg_as_text}"
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing image as base64 string')
        else:
            self.get_logger().error('Could not read image from webcam')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()