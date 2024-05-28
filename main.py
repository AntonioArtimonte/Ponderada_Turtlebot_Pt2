import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
import threading
import pygame
import cv2
import base64
import numpy as np
from PIL import Image
import io
import queue
import time

# Initialize Pygame
pygame.init()

# Set up the display
screen = pygame.display.set_mode((1680, 720))
pygame.display.set_caption("Teleoperacao do Robo + Imagem da Camera")

# Define button properties
button_color = (50, 50, 50)
button_hover_color = (100, 100, 100)
button_text_color = (255, 255, 255)
button_font = pygame.font.Font(None, 36)

button_rects = {
    "kill": pygame.Rect(10, 10, 150, 50),
    "forward": pygame.Rect(910, 500, 150, 50),
    "backward": pygame.Rect(910, 650, 150, 50),
    "left": pygame.Rect(750, 575, 150, 50),
    "right": pygame.Rect(1070, 575, 150, 50),
    "stop": pygame.Rect(910, 575, 150, 50),
}

button_labels = {
    "kill": "Emergência",
    "forward": "Frente",
    "backward": "Atrás",
    "left": "Esquerda",
    "right": "Direita",
    "stop": " ",
}

# Function to draw buttons
def draw_buttons():
    for label, rect in button_rects.items():
        pygame.draw.rect(screen, button_color, rect, border_radius=10)
        text_surf = button_font.render(button_labels[label], True, button_text_color)
        text_rect = text_surf.get_rect(center=rect.center)
        screen.blit(text_surf, text_rect)

# Create a queue to manage UI updates
ui_queue = queue.Queue(maxsize=10)  # Small maximum size to avoid high latency

# Class that controls the robot
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.emergency_client = self.create_client(Empty, 'emergency_stop') # New ROS2 topic made to stop execution of the bringup in the Robot
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.killed = False
        self.safety_distance = 0.35
        self.front_clear = True
        self.back_clear = True

    def lidar_callback(self, msg):
        num_ranges = len(msg.ranges)
        sector_size = num_ranges // 12

        front_left_indices = range(num_ranges - sector_size, num_ranges)
        front_right_indices = range(0, sector_size)
        back_indices = range(5 * sector_size, 7 * sector_size)

        front_ranges = [msg.ranges[i] for i in front_left_indices if 0.01 < msg.ranges[i] < 100.0] + \
                       [msg.ranges[i] for i in front_right_indices if 0.01 < msg.ranges[i] < 100.0]
        back_ranges = [msg.ranges[i] for i in back_indices if 0.01 < msg.ranges[i] < 100.0]

        self.front_clear = not any(r < self.safety_distance for r in front_ranges)
        self.back_clear = not any(r < self.safety_distance for r in back_ranges)

        if not self.front_clear and self.linear_speed > 0:
            self.stop_robot()
        elif not self.back_clear and self.linear_speed < 0:
            self.stop_robot()

    def move_robot(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher.publish(msg)
        print(f"Movendo: velocidade linear={self.linear_speed} m/s, velocidade angular={self.angular_speed} rad/s")

    def stop_robot(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()
        print("Parando o robô.")

    def increase_linear_speed(self):
        if self.front_clear:
            self.linear_speed = 0.1
            self.move_robot()
        else:
            self.stop_robot()

    def decrease_linear_speed(self):
        if self.back_clear:
            self.linear_speed = -0.1
            self.move_robot()
        else:
            self.stop_robot()

    def increase_angular_speed(self):
        self.angular_speed = 0.1
        self.move_robot()

    def decrease_angular_speed(self):
        self.angular_speed = -0.1
        self.move_robot()

    def send_emergency_stop(self):
        if self.emergency_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            future = self.emergency_client.call_async(req)
            future.add_done_callback(self.emergency_stop_callback)
        else:
            print('Emergency service not available, SHUTDOWN THE ROBOTTTT')

    def emergency_stop_callback(self, future):
        try:
            future.result()
            print('Emergency stop signal sent successfully, robot process terminated.')
        except Exception as e:
            print(f'Failed to call emergency stop service: {e}, REMOVE THE BATTERY FROM THE ROBOT!!!')

    def kill_switch(self):
        print("Emergency process stop forced.")
        self.stop_robot()
        self.send_emergency_stop()
        rclpy.shutdown()

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        timestamp, jpg_as_text = msg.data.split('|', 1)
        timestamp = float(timestamp)
        current_time = time.time()
        latency = (current_time - timestamp) * 1000  # Convert to milliseconds

        jpg_original = base64.b64decode(jpg_as_text)
        jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)
        img = cv2.imdecode(jpg_as_np, cv2.IMREAD_COLOR)

        if img is not None:
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(img_rgb)
            img_bytes = io.BytesIO()
            pil_image.save(img_bytes, format="JPEG")
            img_bytes.seek(0)
            if not ui_queue.full():
                ui_queue.put((img_bytes, latency))
                latencies.append(latency)  # Store the latency
        else:
            self.get_logger().error('Could not decode the image')

def init_ros_nodes():
    rclpy.init()
    robot_controller = RobotController()
    listener = Listener()

    executor_thread = threading.Thread(target=spin_nodes, args=(robot_controller, listener), daemon=True)
    executor_thread.start()

    return robot_controller, listener

def spin_nodes(robot_controller, listener):
    while rclpy.ok():
        rclpy.spin_once(robot_controller, timeout_sec=0.01)
        rclpy.spin_once(listener, timeout_sec=0.01)

robot_controller, listener = init_ros_nodes()


latencies = []

def moving_average(latency_list, window_size=10):
    if len(latency_list) < window_size:
        return sum(latency_list) / len(latency_list)
    else:
        return sum(latency_list[-window_size:]) / window_size


def main():
    running = True
    clock = pygame.time.Clock()

    # Font setup
    font = pygame.font.Font(None, 36)
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    robot_controller.increase_linear_speed()
                elif event.key == pygame.K_s:
                    robot_controller.decrease_linear_speed()
                elif event.key == pygame.K_a:
                    robot_controller.increase_angular_speed()
                elif event.key == pygame.K_d:
                    robot_controller.decrease_angular_speed()
                elif event.key == pygame.K_SPACE:
                    robot_controller.stop_robot()
                elif event.key == pygame.K_q:
                    running = False
                elif event.key == pygame.K_b:
                    robot_controller.kill_switch()
                    running = False
            elif event.type == pygame.KEYUP:
                if event.key in [pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d]:
                    robot_controller.stop_robot()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = event.pos
                if button_rects["kill"].collidepoint(mouse_pos):
                    robot_controller.kill_switch()
                    running = False
                elif button_rects["forward"].collidepoint(mouse_pos):
                    robot_controller.increase_linear_speed()
                elif button_rects["backward"].collidepoint(mouse_pos):
                    robot_controller.decrease_linear_speed()
                elif button_rects["left"].collidepoint(mouse_pos):
                    robot_controller.increase_angular_speed()
                elif button_rects["right"].collidepoint(mouse_pos):
                    robot_controller.decrease_angular_speed()
                elif button_rects["stop"].collidepoint(mouse_pos):
                    robot_controller.stop_robot()

        if not ui_queue.empty():
            img_bytes, latency = ui_queue.get()
            img_np = np.array(Image.open(img_bytes))
            img_np = np.rot90(img_np, 1)
            img_np = cv2.resize(img_np, (720, 1680))
            img_surface = pygame.surfarray.make_surface(img_np)
            screen.blit(img_surface, (0, 0))

            # Display latency text
            avg_latency = moving_average(latencies)
            latency_text = font.render(f"Latência Média: {avg_latency:.2f} ms", True, (255, 255, 255))
            screen.blit(latency_text, (1300, 10))  # Adjust position as needed

        # Display linear and angular speeds
        linear_speed_text = font.render(f"Vel. Linear: {robot_controller.linear_speed:.2f} m/s", True, (255, 255, 255))
        angular_speed_text = font.render(f"Vel. Angular: {robot_controller.angular_speed:.2f} rad/s", True, (255, 255, 255))
        screen.blit(linear_speed_text, (1300, 50))
        screen.blit(angular_speed_text, (1300, 90))
        
        draw_buttons()
        pygame.display.flip()
        
        clock.tick(60)


    pygame.quit()

if __name__ == "__main__":
    main()
