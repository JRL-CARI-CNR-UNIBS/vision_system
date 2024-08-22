
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

import pytest
import rclpy
from rclpy.node import Node
from rosbags.rosbag2 import Reader
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from vision_system.camera import Camera
import os
import threading, time
import numpy as np
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from rosbags.typesys import Stores, get_typestore

COLOR_FRAME_TOPIC_NAME_TEST = '/head_front_camera/rgb/image_raw'
DEPTH_FRAME_TOPIC_NAME_TEST = '/head_front_camera/depth_registered/image_raw'
CAMERA_INFO_TOPIC_NAME_TEST = '/head_front_camera/depth_registered/camera_info'

class FakeCamera(Node):
    def __init__(self):
        super().__init__('bag_publisher')
        #define a qos transient local
        # self.qos = QoSProfile(depth=10)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.camera_info_pub = self.create_publisher(CameraInfo, CAMERA_INFO_TOPIC_NAME_TEST, 10)
        self.color_image_pub = self.create_publisher(Image, COLOR_FRAME_TOPIC_NAME_TEST, 10)
        self.depth_image_pub = self.create_publisher(Image, DEPTH_FRAME_TOPIC_NAME_TEST, 10)
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        bag_path = os.path.join(current_dir, 'test_bag')  

        self.camera_info_msg = None
        self.color_image_msg = None
        self.depth_image_msg = None

        typestore = get_typestore(Stores.ROS2_HUMBLE)
        with Reader(bag_path) as reader:
          reader.open()

          timestamp_msg = self.get_clock().now().to_msg()
          for connection, timestamp, rawdata in reader.messages():
              if connection.topic == CAMERA_INFO_TOPIC_NAME_TEST and self.camera_info_msg is None:
                  camera_info = typestore.deserialize_cdr(rawdata, connection.msgtype)
                  self.camera_info_msg = CameraInfo()
                  self.camera_info_msg.header = Header()
                  self.camera_info_msg.header.stamp = timestamp_msg
                  self.camera_info_msg.header.frame_id = camera_info.header.frame_id
                  self.camera_info_msg.height = camera_info.height
                  self.camera_info_msg.width = camera_info.width
                  self.camera_info_msg.distortion_model = camera_info.distortion_model
                  self.camera_info_msg.d = [float(value) for value in camera_info.d]
                  self.camera_info_msg.k = camera_info.k
                  self.camera_info_msg.r = camera_info.r
                  self.camera_info_msg.p = camera_info.p
                  self.camera_info_msg.binning_x = camera_info.binning_x
                  self.camera_info_msg.binning_y = camera_info.binning_y
              elif connection.topic == COLOR_FRAME_TOPIC_NAME_TEST and self.color_image_msg is None:
                  color_image = typestore.deserialize_cdr(rawdata, connection.msgtype)
                  self.color_image_msg = Image()
                  self.color_image_msg.header = Header()
                  self.color_image_msg.header.stamp = timestamp_msg
                  self.color_image_msg.header.frame_id = color_image.header.frame_id
                  self.color_image_msg.height = color_image.height
                  self.color_image_msg.width = color_image.width
                  self.color_image_msg.encoding = color_image.encoding
                  self.color_image_msg.is_bigendian = color_image.is_bigendian
                  self.color_image_msg.step = color_image.step
                  self.color_image_msg.data = [int(value) for value in color_image.data]
              elif connection.topic == DEPTH_FRAME_TOPIC_NAME_TEST and self.depth_image_msg is None:
                  depth_image = typestore.deserialize_cdr(rawdata, connection.msgtype)
                  self.depth_image_msg = Image()
                  self.depth_image_msg.header = Header()
                  self.depth_image_msg.header.stamp = timestamp_msg
                  self.depth_image_msg.header.frame_id = depth_image.header.frame_id
                  self.depth_image_msg.height = depth_image.height
                  self.depth_image_msg.width = depth_image.width
                  self.depth_image_msg.encoding = depth_image.encoding
                  self.depth_image_msg.is_bigendian = depth_image.is_bigendian
                  self.depth_image_msg.step = depth_image.step
                  self.depth_image_msg.data = [int(value) for value in depth_image.data]

    def publish_messages(self, wait: bool = False):
        self._publish(wait)

    def publish_messages_after_a_while(self):
        pub_thread = threading.Thread(target=self.publish_messages, args=(True,))
        pub_thread.start()

    def publish_camera_info_after_a_while(self):
        pub_thread = threading.Thread(target=self.publish_camera_info)
        pub_thread.start()
        
    def publish_camera_info(self):
        time.sleep(0.3)
        timestamp = self.get_clock().now().to_msg()
        self.camera_info_msg.header.stamp = timestamp
        self.camera_info_pub.publish(self.camera_info_msg)

    def _publish(self, wait: bool = False):
        if wait:
          time.sleep(0.3)
        timestamp = self.get_clock().now().to_msg()
        self.camera_info_msg.header.stamp = timestamp
        self.color_image_msg.header.stamp = timestamp
        self.depth_image_msg.header.stamp = timestamp
        # Publish msgs
        self.camera_info_pub.publish(self.camera_info_msg)
        self.color_image_pub.publish(self.color_image_msg)
        self.depth_image_pub.publish(self.depth_image_msg)

@pytest.mark.dependency(name="setUp")
def test_setup():
    rclpy.init()

@pytest.fixture
def fake_camera():
    return FakeCamera()

@pytest.mark.dependency(name="camera_as_class_color", depends=["setUp"])
def test_camera_as_class_color_frame(fake_camera):
    camera = Camera(
        color_image_topic=COLOR_FRAME_TOPIC_NAME_TEST,
        depth_image_topic=DEPTH_FRAME_TOPIC_NAME_TEST,
        camera_info_topic=CAMERA_INFO_TOPIC_NAME_TEST
    )
    fake_camera.publish_messages_after_a_while()
    color_frame = camera.acquire_color_frame_once()
     
    # Assert that the color frame is not None
    assert color_frame is not None, "The color frame should not be None"

    # Assert that the color frame is of type Image
    assert isinstance(color_frame, np.ndarray), "The color frame should be of type Image"

@pytest.mark.dependency(name="camera_as_class_color_and_depth", depends=["setUp", "camera_as_class_color"])
def test_camera_as_class_color_and_depth_frames(fake_camera):
    camera = Camera(
        color_image_topic=COLOR_FRAME_TOPIC_NAME_TEST,
        depth_image_topic=DEPTH_FRAME_TOPIC_NAME_TEST,
        camera_info_topic=CAMERA_INFO_TOPIC_NAME_TEST
    )
    # fake_camera.publish_messages_after_a_while()
    fake_camera.publish_messages_after_a_while()  # Is necessary to publish the messages twice since color wait for message spin consume also depth message

    color_frame, distance_frame = camera.acquire_frames_once()
    
    # Assert that the color frame is not None
    assert color_frame is not None, "The color frame should not be None"
    assert distance_frame is not None, "The distance_frame should not be None"

    # Assert that the color frame is of type Image
    assert isinstance(color_frame, np.ndarray), "The color frame should be of type Image"
    assert isinstance(distance_frame, np.ndarray), "The distance_frame should be of type Image"

@pytest.mark.dependency(name="camera_post_processing_loading", depends=["setUp", "camera_as_class_color", "camera_as_class_color_and_depth"])
def test_camera_post_processing_loading():
    camera = Camera(
        color_image_topic=COLOR_FRAME_TOPIC_NAME_TEST,
        depth_image_topic=DEPTH_FRAME_TOPIC_NAME_TEST,
        camera_info_topic=CAMERA_INFO_TOPIC_NAME_TEST
    )
    with pytest.raises(Exception):
      camera.set_processing_function(package_name='wrong',
                                module_name='wrong',
                                class_name='wrong')

@pytest.mark.dependency(name="camera_post_processing", depends=["setUp", "camera_as_class_color", "camera_as_class_color_and_depth", "camera_post_processing_loading"])
def test_camera_as_class_with_post_processing(capsys, fake_camera):
    camera = Camera(
        color_image_topic=COLOR_FRAME_TOPIC_NAME_TEST,
        depth_image_topic=DEPTH_FRAME_TOPIC_NAME_TEST,
        camera_info_topic=CAMERA_INFO_TOPIC_NAME_TEST
    )
    
    try:
      camera.set_processing_function(package_name='test',
                                module_name='post_processing_example',
                                class_name='YOLOMock')
    except (Exception, TypeError):
        pytest.fail("The set_processing_function should not raise an exception")
    fake_camera.publish_messages_after_a_while()
    camera.process_once()
    captured = capsys.readouterr()
    assert captured.out == 'Put post processing code here.\nExample of post processing: (480, 640, 3), (480, 640)\n'

@pytest.mark.dependency(name="camera_node_parameters", depends=["setUp", "camera_post_processing"])
def test_camera_node_parameters():
    camera = Camera()
    camera.set_parameters([rclpy.parameter.Parameter('color_image_topic', rclpy.Parameter.Type.STRING, COLOR_FRAME_TOPIC_NAME_TEST),
                           rclpy.parameter.Parameter('depth_image_topic', rclpy.Parameter.Type.STRING, DEPTH_FRAME_TOPIC_NAME_TEST),
                           rclpy.parameter.Parameter('camera_info_topic', rclpy.Parameter.Type.STRING, CAMERA_INFO_TOPIC_NAME_TEST)])
    assert camera.get_parameter('color_image_topic').get_parameter_value().string_value == COLOR_FRAME_TOPIC_NAME_TEST
    assert camera.get_parameter('depth_image_topic').get_parameter_value().string_value == DEPTH_FRAME_TOPIC_NAME_TEST
    assert camera.get_parameter('camera_info_topic').get_parameter_value().string_value == CAMERA_INFO_TOPIC_NAME_TEST

@pytest.mark.dependency(name="camera_node", depends=["setUp", "camera_node_parameters"])
def test_camera_node(fake_camera):
    camera = Camera()

    camera = Camera(
        color_image_topic=COLOR_FRAME_TOPIC_NAME_TEST,
        depth_image_topic=DEPTH_FRAME_TOPIC_NAME_TEST,
        camera_info_topic=CAMERA_INFO_TOPIC_NAME_TEST
    )

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(camera)
    executor.add_node(fake_camera)
    
    t_start = camera.get_clock().now().nanoseconds * 1e-9
    elapsed_time = 0.0

    fake_camera.publish_camera_info_after_a_while()
    camera.start_acquire()
    
    while rclpy.ok() and elapsed_time < 2:
        fake_camera.publish_messages()
        executor.spin_once(timeout_sec=0.1)

        elapsed_time = camera.get_clock().now().nanoseconds * 1e-9 - t_start

    color_frame = camera.get_color_frame()
    distance_frame = camera.get_distance_frame()
    frame_id = camera.get_frame_id()
    assert color_frame is not None, "The color frame should not be None"
    assert distance_frame is not None, "The distance_frame should not be None"
    assert isinstance(color_frame, np.ndarray), "The color frame should be of type Image"
    assert isinstance(distance_frame, np.ndarray), "The distance_frame should be of type Image"
    assert frame_id == 'head_front_camera_rgb_optical_frame', "The frame_id should be head_front_camera_link"
    
@pytest.mark.dependency(name="camera_node_loop", depends=["setUp", "camera_node"])
def test_camera_node_in_loop(fake_camera):
    camera = Camera()

    camera = Camera(
        color_image_topic=COLOR_FRAME_TOPIC_NAME_TEST,
        depth_image_topic=DEPTH_FRAME_TOPIC_NAME_TEST,
        camera_info_topic=CAMERA_INFO_TOPIC_NAME_TEST
    )

    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(camera)
    executor.add_node(fake_camera)
    
    t_start = camera.get_clock().now().nanoseconds * 1e-9
    elapsed_time = 0.0

    fake_camera.publish_camera_info_after_a_while()
    camera.start_acquire()
    
    test_node = Node('test_node')
    executor.add_node(test_node)
    rate = test_node.create_rate(10)
    spin_thread = threading.Thread(target=executor.spin, args=())
    spin_thread.start()

    all_color_frame_none = True
    all_distance_frame_none = True 
    all_frame_id_none = True
    while rclpy.ok() and elapsed_time < 2:
        fake_camera.publish_messages()
        
        color_frame = camera.get_color_frame()
        distance_frame = camera.get_distance_frame()
        frame_id = camera.get_frame_id()
        if color_frame is not None:
            all_color_frame_none = False
        if distance_frame is not None:
            all_distance_frame_none = False
        if frame_id is not None:
            all_frame_id_none = False

        rate.sleep()
        elapsed_time = camera.get_clock().now().nanoseconds * 1e-9 - t_start

    executor.shutdown()
    spin_thread.join()
    assert not all_color_frame_none, "The color frame should not be None"
    assert not all_distance_frame_none, "The distance_frame should not be None"
    assert not all_frame_id_none, "The frame_id should not be None"    

@pytest.mark.dependency(name="camera_node_loop_with_processing", depends=["setUp", "camera_node_loop"])
def test_camera_node_in_loop_with_processing(capsys, fake_camera):
    camera = Camera()

    camera = Camera(
        color_image_topic=COLOR_FRAME_TOPIC_NAME_TEST,
        depth_image_topic=DEPTH_FRAME_TOPIC_NAME_TEST,
        camera_info_topic=CAMERA_INFO_TOPIC_NAME_TEST
    )

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(camera)
    executor.add_node(fake_camera)
    
    t_start = camera.get_clock().now().nanoseconds * 1e-9
    elapsed_time = 0.0

    fake_camera.publish_camera_info_after_a_while()
    camera.retrieve_camera_info()
    try:
      camera.set_processing_function(package_name='test',
                                module_name='post_processing_example',
                                class_name='YOLOMock')
    except (Exception, TypeError):
        pytest.fail("The set_processing_function should not raise an exception")

    
    test_node = Node('test_node')

    test_executor = rclpy.executors.SingleThreadedExecutor()
    test_executor.add_node(test_node)
    rate = test_node.create_rate(10)

    spin_thread = threading.Thread(target=test_executor.spin, args=())
    spin_thread.start()

    still_first_acquisition = True
    camera.start_acquire()
    
    while rclpy.ok() and elapsed_time < 2 and still_first_acquisition:
        fake_camera.publish_messages()
        executor.spin_once()        
        color_frame = camera.get_color_frame()
        if color_frame is not None:
            still_first_acquisition = False

        rate.sleep()
        elapsed_time = camera.get_clock().now().nanoseconds * 1e-9 - t_start

    executor.shutdown()
    test_executor.shutdown()
    spin_thread.join()
    captured = capsys.readouterr()
    assert captured.out == 'Put post processing code here.\nExample of post processing: (480, 640, 3), (480, 640)\n'
