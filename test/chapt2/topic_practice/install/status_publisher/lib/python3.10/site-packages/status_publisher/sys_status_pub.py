import rclpy
from status_interfaces.msg import SystemStatus
from rclpy.node import Node
import psutil
import platform

class SystemStatusPublisher(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.status_publisher_ = self.create_publisher(SystemStatus, 'sys_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        cpu_percent = psutil.cpu_percent(interval=1.0)
        memory_info = psutil.virtual_memory()
        net_io_counters = psutil.net_io_counters()

        msg = SystemStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.host_name = platform.node()
        msg.cpu_percentage = cpu_percent
        msg.memory_percentage = memory_info.percent
        msg.memory_total = float(memory_info.total)
        msg.memory_available = float(memory_info.available)
        msg.net_send = net_io_counters.bytes_sent / 1024.0 / 1024.0
        msg.net_recv = net_io_counters.bytes_recv / 1024.0 / 1024.0
        
        self.status_publisher_.publish(msg)
        self.get_logger().info(f'发布: {str(msg)}')

def main():
    rclpy.init()
    node = SystemStatusPublisher('sys_status_pub')
    rclpy.spin(node)
    rclpy.shutdown()