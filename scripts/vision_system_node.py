import rclpy
from vision_system.camera import Camera

def main():
  rclpy.init()
  vision_system_node = Camera()
  vision_system_node.retrieve_camera_info()
  
  vision_system_node.acquire()
  rclpy.spin(vision_system_node)
  
  vision_system_node.destroy_node()
  rclpy.shutdown()