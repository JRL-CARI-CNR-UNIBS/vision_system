
# Copyright 2024 National Research Council STIIMA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from vision_system.camera import Camera

def main():
  rclpy.init()
  vision_system_node = Camera()  
  vision_system_node.retrieve_camera_info()
  
  rclpy.spin(vision_system_node)
  
  vision_system_node.destroy_node()
  rclpy.shutdown()