import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import subprocess
import signal
import os

class TurtlebotBringupManager(Node):
    def __init__(self):
        super().__init__('turtlebot_bringup_manager')
        self.srv = self.create_service(Empty, 'emergency_stop', self.emergency_stop_callback)
        self.bringup_process = None

    def start_bringup(self):
        self.bringup_process = subprocess.Popen(['ros2', 'launch', 'turtlebot3_bringup', 'robot.launch.py'])

    def emergency_stop_callback(self, request, response):
        self.get_logger().info('Emergency stop received. Stopping TurtleBot3 bringup.')
        if self.bringup_process:
            os.killpg(os.getpgid(self.bringup_process.pid), signal.SIGTERM)
            self.bringup_process = None
        return response

def main(args=None):
    rclpy.init(args=args)
    manager = TurtlebotBringupManager()
    manager.start_bringup()
    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()