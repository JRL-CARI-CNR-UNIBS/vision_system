
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


import importlib
import message_filters
from cv_bridge import CvBridge, CvBridgeError

from rclpy.node import Node
import rclpy.wait_for_message

from sensor_msgs.msg import CameraInfo, Image
from vision_system.post_processing import PostProcessing

DEFAULT_COLOR_IMAGE_TOPIC = '/camera/color/image_raw'
DEFAULT_DEPTH_IMAGE_TOPIC = '/camera/depth/image_raw'
DEFAULT_CAMERA_INFO_TOPIC = '/camera/depth/camera_info'
DEFAULT_FRAMES_APPROX_SYNC = False
DEFAULT_DEPTH_FRAME_ENCODING = '32FC1'

class Camera(Node):
    """
    RealSense class for Subscribe interesting topic.
    """

    def __init__(self, 
                 color_image_topic = DEFAULT_COLOR_IMAGE_TOPIC, 
                 depth_image_topic = DEFAULT_DEPTH_IMAGE_TOPIC, 
                 camera_info_topic = DEFAULT_CAMERA_INFO_TOPIC, 
                 frames_approx_sync = DEFAULT_FRAMES_APPROX_SYNC,
                 depth_frame_encoding = DEFAULT_DEPTH_FRAME_ENCODING):

        super().__init__('camera_node')

        # Declare parameters
        self.declare_parameter('color_image_topic', color_image_topic)
        self.declare_parameter('depth_image_topic', depth_image_topic)
        self.declare_parameter('camera_info_topic', camera_info_topic)
        self.declare_parameter('frames_approx_sync', frames_approx_sync)
        self.declare_parameter('depth_frame_encoding', depth_frame_encoding)

        # Get parameter values
        self.color_image_topic = self.get_parameter('color_image_topic').get_parameter_value().string_value
        self.depth_image_topic = self.get_parameter('depth_image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.frames_approx_sync = self.get_parameter('frames_approx_sync').get_parameter_value().bool_value
        self.depth_frame_encoding = self.get_parameter('depth_frame_encoding').get_parameter_value().string_value

        self._cv_bridge = CvBridge()

        self.color_frame = None
        self.depth_frame = None
        self.distance_frame = None
        self.intrinsics = None
        self.camera_info = None

        self.post_processing: PostProcessing = None

        self.image_sub = message_filters.Subscriber(
            self, Image, color_image_topic)
        self.depth_sub = message_filters.Subscriber(
            self, Image, depth_image_topic)
        
        if frames_approx_sync:
          self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (self.image_sub, self.depth_sub), 10, 0.5)
        else:
          self._synchronizer = message_filters.TimeSynchronizer(
            (self.image_sub, self.depth_sub), 10)
        
    
    def _convert_frames(self, color_frame, depth_frame):
      try:
        self.color_frame = self._cv_bridge.imgmsg_to_cv2(color_frame, desired_encoding='passthrough')
        self.depth_frame = self._cv_bridge.imgmsg_to_cv2(depth_frame, desired_encoding='passthrough')
        self.distance_frame = self._cv_bridge.imgmsg_to_cv2(depth_frame, desired_encoding=self.depth_frame_encoding)
      except CvBridgeError as e:
        self.get_logger().error(f'Error converting image: {e}')
        return False
      return True
      
    def retrieve_camera_info(self):
      self.get_logger().info('Waiting for camera info...')
      retrieved, self.camera_info = rclpy.wait_for_message.wait_for_message(
            CameraInfo, self, self.camera_info_topic, time_to_wait=30.0
        )
      if not retrieved:
        self.get_logger().error('Failed to retrieve camera info.')
        return False
      self.get_logger().info('Camera info retrieved.')
      return True
        
    def acquire_color_frame_once(self):
      retrieved, color_frame = rclpy.wait_for_message.wait_for_message(
            Image, self, self.color_image_topic, time_to_wait=3.0
        )
      if not retrieved:
        self.get_logger().error('Failed to retrieve image.')
        return None
      
      try:
        self.color_frame = self._cv_bridge.imgmsg_to_cv2(color_frame, desired_encoding="passthrough")
      except CvBridgeError as e:
        self.get_logger().error(f'Error converting image: {e}')
        return None
      
      return self.color_frame.copy()

    def acquire_frames_once(self):
      retrieved_color, color_frame = rclpy.wait_for_message.wait_for_message(
            Image, self, self.color_image_topic, time_to_wait=3
        )
      retrieved_depth, depth_frame = rclpy.wait_for_message.wait_for_message(
            Image, self, self.depth_image_topic, time_to_wait=3
        )
      
      if not retrieved_color or not retrieved_depth:
        self.get_logger().error('Failed to retrieve frames.')
        return None, None

      if not self._convert_frames(color_frame, depth_frame):
        return None, None

      return self.color_frame.copy(), self.distance_frame.copy()
    
    def process_once(self):
      color_frame, distance_frame = self.acquire_frames_once()
      if color_frame is None or distance_frame is None:
        return None
      if self.post_processing is not None:
        self.post_processing.process_frames(color_frame, distance_frame)
           
    def set_processing_function(self, package_name, module_name, class_name):
      try:
          module = importlib.import_module(f'{package_name}.{module_name}')
          
          post_processor = getattr(module, class_name)()
          if not isinstance(post_processor, PostProcessing):
            raise TypeError(f'Extractor {class_name} is not an instance of PostProcessing')
          self.post_processing = post_processor 
      except Exception as e:
          self.get_logger().error(f'Error importing proccessing function: {e}')
          raise e

    def acquire(self):
      if self.camera_info is None:
        self.retrieve_camera_info()
      self._synchronizer.registerCallback(self._process)
    
    def _process(self, color_frame: Image, depth_frame: Image):
      if not self._convert_frames(color_frame, depth_frame):
        return None

      if self.post_processing is not None:
        self.post_processing.process_frames(self.color_frame, self.distance_frame)

      return self.color_frame.copy(), self.distance_frame.copy()

    def _process_color_frame(self, color_frame: Image):
      try:
        self.color_frame = self._cv_bridge.imgmsg_to_cv2(color_frame, desired_encoding="passthrough")
      except CvBridgeError as e:
        self.get_logger().error(f'Error converting image: {e}')
        return None

      if self.post_processing is not None:
        self.post_processing.process_frame(self.color_frame)

      return self.color_frame.copy(), self.distance_frame.copy()

    def acquire_only_color(self):
      self.color_sub= self.create_subscription(
            Image,
            self.color_image_topic,
            self._process_color_frame,
            10
        )

    def get_frames(self):
      return self.color_frame.copy(), self.distance_frame.copy()
  
    def get_color_frame(self):
      return self.color_frame.copy()
    
    def get_distance_frame(self):
      return self.distance_frame.copy()

    def get_frame_id(self):
      #TODO(@kalman): Implement
      pass