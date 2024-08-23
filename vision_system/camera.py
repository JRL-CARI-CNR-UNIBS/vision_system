
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
import numpy as np
import copy
from typing import Optional, Tuple

DEFAULT_COLOR_IMAGE_TOPIC = '/camera/color/image_raw'
DEFAULT_DEPTH_IMAGE_TOPIC = '/camera/depth/image_raw'
DEFAULT_CAMERA_INFO_TOPIC = '/camera/depth/camera_info'
DEFAULT_FRAMES_APPROX_SYNC = False
DEFAULT_DEPTH_FRAME_ENCODING = '32FC1'

class Camera(Node):
    """
    Camera class for acquiring images (color and depth) from an RGBD-camera.

    This class is responsible for subscribing to the image and depth topics making it easy to access to camera frames,
    converting the images using `cv_bridge`, and optionally applying post-processing.

    Parameters
    ----------
    color_image_topic : str, optional
        The topic for color images (default is '/camera/color/image_raw').
    depth_image_topic : str, optional
        The topic for depth images (default is '/camera/depth/image_raw').
    camera_info_topic : str, optional
        The topic for camera information (default is '/camera/depth/camera_info').
    frames_approx_sync : bool, optional
        Whether to use approximate time synchronization (default is False).
    depth_frame_encoding : str, optional
        The encoding of the depth frame (default is '32FC1').
    """

    def __init__(self, 
                 color_image_topic: str = DEFAULT_COLOR_IMAGE_TOPIC, 
                 depth_image_topic: str = DEFAULT_DEPTH_IMAGE_TOPIC, 
                 camera_info_topic: str = DEFAULT_CAMERA_INFO_TOPIC, 
                 frames_approx_sync: bool = DEFAULT_FRAMES_APPROX_SYNC,
                 depth_frame_encoding: str = DEFAULT_DEPTH_FRAME_ENCODING,
                 ):

        super().__init__('camera_node')

        # Declare parameters
        self.declare_parameter('color_image_topic', color_image_topic)
        self.declare_parameter('depth_image_topic', depth_image_topic)
        self.declare_parameter('camera_info_topic', camera_info_topic)
        self.declare_parameter('frames_approx_sync', frames_approx_sync)
        self.declare_parameter('depth_frame_encoding', depth_frame_encoding)
        self.declare_parameter('post_processing.package', '')
        self.declare_parameter('post_processing.module', '')
        self.declare_parameter('post_processing.class', '')

        # Get parameter values
        self.color_image_topic = self.get_parameter('color_image_topic').get_parameter_value().string_value
        self.depth_image_topic = self.get_parameter('depth_image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.frames_approx_sync = self.get_parameter('frames_approx_sync').get_parameter_value().bool_value
        self.depth_frame_encoding = self.get_parameter('depth_frame_encoding').get_parameter_value().string_value

        self._cv_bridge = CvBridge()

        self.color_frame: Optional[np.ndarray] = None
        self.depth_frame: Optional[np.ndarray] = None
        self.distance_frame: Optional[np.ndarray] = None
        self.intrinsics = None
        self.camera_info: Optional[CameraInfo] = None

        self.post_processing: Optional[PostProcessing] = None

        self.image_sub = message_filters.Subscriber(
            self, Image, self.color_image_topic)
        self.depth_sub = message_filters.Subscriber(
            self, Image, self.depth_image_topic)

        if self.frames_approx_sync:
          self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (self.image_sub, self.depth_sub), 10, 0.5)
        else:
          self._synchronizer = message_filters.TimeSynchronizer(
            (self.image_sub, self.depth_sub), 10)
        
        post_processing_package = self.get_parameter('post_processing.package').get_parameter_value().string_value
        post_processing_module = self.get_parameter('post_processing.module').get_parameter_value().string_value
        post_processing_class = self.get_parameter('post_processing.class').get_parameter_value().string_value

        if post_processing_package and post_processing_module and post_processing_class:
          self.get_logger().info(post_processing_package)
          try:
            self.set_processing_function(post_processing_package, 
                                         post_processing_module, 
                                         post_processing_class)        
          except (Exception, TypeError) as e:
            self.get_logger().warning(f'Error setting processing function: {e}. It will not be called')

    def _convert_frames(self, color_frame: Image, depth_frame: Image) -> bool:
      try:
        self.color_frame = self._cv_bridge.imgmsg_to_cv2(color_frame, desired_encoding='passthrough')
        self.depth_frame = self._cv_bridge.imgmsg_to_cv2(depth_frame, desired_encoding='passthrough')
        self.distance_frame = self._cv_bridge.imgmsg_to_cv2(depth_frame, desired_encoding=self.depth_frame_encoding)
      except CvBridgeError as e:
        self.get_logger().error(f'Error converting image: {e}')
        return False
      return True
      
    def retrieve_camera_info(self) -> bool:
      """
      Retrieves the camera info waiting for the topic and stores it in the class.

      :return: True if camera info is successfully retrieved, False otherwise.
      """
      self.get_logger().info('Waiting for camera info...')
      retrieved, self.camera_info = rclpy.wait_for_message.wait_for_message(
            CameraInfo, self, self.camera_info_topic, time_to_wait=3.0
        )
      if not retrieved:
        self.get_logger().error('Failed to retrieve camera info.')
        return False
      self.get_logger().info('Camera info retrieved.')
      return True
        
    def acquire_color_frame_once(self) -> Optional[np.ndarray]:
      """
      Acquires a single color frame waiting for the topic.

      :return: The acquired color frame as a cv2 image as NumPy array, or None if acquisition fails.
      """
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

    def acquire_frames_once(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
      """
      Acquires a single pair of color and depth frames.

      :return: A tuple of (color_frame, depth_frame) as NumPy arrays, or (None, None) if acquisition fails.
      """
      self._reset_frames()
      image_sub = message_filters.Subscriber(
        self, Image, self.color_image_topic)
      depth_sub = message_filters.Subscriber(
        self, Image, self.depth_image_topic)
      
      def acquire_frames_once_callback(color_frame: Image, depth_frame: Image) -> None:
        self._convert_frames(color_frame, depth_frame)
        self.destroy_subscription(image_sub)
        self.destroy_subscription(depth_sub)
      
      synchronizer = message_filters.TimeSynchronizer(
          (image_sub, depth_sub), 10)
      synchronizer.registerCallback(acquire_frames_once_callback)

      t_start = self.get_clock().now().nanoseconds*1e-9
      elapsed_time = 0.0
      while rclpy.ok() and self.color_frame is None and elapsed_time < 3:
          rclpy.spin_once(self, timeout_sec=0.1)
          elapsed_time = self.get_clock().now().nanoseconds*1e-9 - t_start
      
      if self.color_frame is not None and self.distance_frame is not None:
        return self.color_frame.copy(), self.distance_frame.copy()
      else:
        return None, None
    
    def process_once(self) -> Optional[np.ndarray]:
      """
      Processes a single pair of color and depth frames using the provided post-processing function.

      :return: Processed frames or None if acquisition or processing fails.
      """
      color_frame, distance_frame = self.acquire_frames_once()
      if color_frame is None or distance_frame is None:
        return None
      if self.post_processing is not None:
        self.post_processing.process_frames(color_frame, distance_frame)
           
    def set_processing_function(self, package_name: str, module_name: str, class_name: str) -> None:
      """
      Dynamically loads and sets the post-processing function.

      :param package_name: The name of the package containing the post-processing module.
      :param module_name: The name of the module containing the post-processing class.
      :param class_name: The name of the post-processing class.
      :raises: Exception if loading fails or if the class is not an instance of PostProcessing.
      """
      try:
          module = importlib.import_module(f'{package_name}.{module_name}')
          
          post_processor = getattr(module, class_name)()
          if not isinstance(post_processor, PostProcessing):
            raise TypeError(f'Extractor {class_name} is not an instance of PostProcessing')
          self.post_processing = post_processor 
      except Exception as e:
          self.get_logger().error(f'Error importing proccessing function: {e}')
          raise e

    def start_acquire(self) -> None:
      """
      Starts the acquisition of synchronized color and depth frames, with post-processing.
      """
      self.get_logger().info('Acquisition started...')
      if self.camera_info is None:
        self.retrieve_camera_info()
      self._synchronizer.registerCallback(self._process)
    
    def _process(self, color_frame: Image, depth_frame: Image):
      if not self._convert_frames(color_frame, depth_frame):
        return
      if self.post_processing is not None:
        self.post_processing.process_frames(self.color_frame.copy(), self.distance_frame.copy())

    def _process_color_frame(self, color_frame: Image):
      try:
        self.color_frame = self._cv_bridge.imgmsg_to_cv2(color_frame, desired_encoding="passthrough")
      except CvBridgeError as e:
        self.get_logger().error(f'Error converting image: {e}')
        return None

      if self.post_processing is not None:
        self.post_processing.process_frame(self.color_frame)

      return self.color_frame.copy(), self.distance_frame.copy()

    def start_acquire_only_color(self) -> None:
      """
      Starts the acquisition of color frames only, with post-processing (is necessary to spin the node).
      """
      self.color_sub= self.create_subscription(
            Image,
            self.color_image_topic,
            self._process_color_frame,
            10
        )

    def get_frames(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
      """
      Retrieves the most recently acquired color and depth frames.

      :return: A tuple of (color_frame, depth_frame) as NumPy arrays, or None if frames are not available.
      """
      if self.color_frame is not None and self.distance_frame is not None:
        return self.color_frame.copy(), self.distance_frame.copy()
  
    def get_color_frame(self) -> Optional[np.ndarray]:
      """
      Retrieves the most recently acquired color frame.

      :return: The color frame as a NumPy array, or None if no frame is available.
      """
      if self.color_frame is not None:
        return self.color_frame.copy()
      return None
    
    def get_distance_frame(self) -> Optional[np.ndarray]:
      """
      Retrieves the most recently acquired depth frame.

      :return: The depth frame as a NumPy array, or None if no frame is available.
      """
      if self.distance_frame is not None:
        return self.distance_frame.copy()
      return None
    
    def get_camera_info(self) -> Optional[CameraInfo]:
      """
      Retrieves the camera info.

      :return: The camera info as a CameraInfo object, or None if it has not been retrieved.
      """
      if self.camera_info is not None:
        return copy.deepcopy(self.camera_info) 
      return None
    
    def get_frame_id(self) -> Optional[str]:
      """
      Retrieves the frame ID from the camera info.

      :return: The frame ID as a string, or None if camera info is not available.
      """
      if self.camera_info is not None:
        return self.camera_info.header.frame_id
      return None

    def _reset_frames(self):
      self.color_frame = None
      self.depth_frame = None
      self.distance_frame = None