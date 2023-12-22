#! /usr/bin/env python3

import rospy
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

import cv2
import numpy as np
import pyrealsense2 as rs2
from dataclasses import dataclass, field
from typing import Optional


# Realsense Topic
COLOR_FRAME_TOPIC = '/camera/color/image_raw'
DEPTH_ALIGNED_TOPIC = '/camera/aligned_depth_to_color/image_raw'
CAMERA_INFO_TOPIC = '/camera/aligned_depth_to_color/camera_info'

CAMERA_FRAME = "camera_color_optical_frame"

# Costant loginfo
PARAMETERS_LOG = 'Camera Parameters acquired \n  Parameters:{}'

@dataclass
class Realsense:
    """
    RealSense class for subscribing to interesting topics.
    """

    color_frame_topic_name: str = COLOR_FRAME_TOPIC
    depth_frame_topic_name: str = DEPTH_ALIGNED_TOPIC
    camera_info_topic_name: str = CAMERA_INFO_TOPIC
    camera_frame_name: str = CAMERA_FRAME

    bridge: CvBridge = field(init=False, default_factory=CvBridge)
    colorFrame: Optional[np.ndarray] = field(init=False, default=None)
    depthFrame: Optional[np.ndarray] = field(init=False, default=None)
    frame_distance: Optional[np.ndarray] = field(init=False, default=None)

    # Camera management
    intrinsics: rs2.intrinsics = field(init=False, default=None)
    cameraInfoReceived: bool = field(init=False, default=False)
    frameAcquired: bool = field(init=False, default=False)

    frame_number: int = field(init=False, default=0)
    
# class Realsense():
#     """
#     RealSense class.
#     """

#     def __init__(self, 
#                  color_frame_topic_name = COLOR_FRAME_TOPIC,
#                  depth_frame_topic_name = DEPTH_ALIGNED_TOPIC,
#                  camera_info_topic_name = CAMERA_INFO_TOPIC,
#                  camera_frame_name = CAMERA_FRAME):
#         """
#         Class builder
#         @param -
#         @return RealSense RealSense object
#         """
#         self.bridge = CvBridge()
#         self.colorFrame = None
#         self.depthFrame = None
#         self.frame_distance = None

#         # Gestione camera pyrealsense2
#         self.intrinsics = None
#         self.cameraInfoReceived = False
#         self.frameAcquired = False

#         self.frame_number = 0

    def initialize(self):
        rospy.loginfo("Waiting, retriving parameters procedure")
        self.getCameraParam()
        self.waitCameraInfo()
        rospy.loginfo("Camera parameters retrived correctly")
        
    def callback(self, frameRgb, frameDepth):
        """
        Callback method to retrieve the content of the topic and convert it in cv2 format. Identify human KeyPoints.
        @param frameRgb : camera msg rgb
        @param frameDepth : camera msg depth
        """
        # Convertion from ros msg image to cv2 image
        colorFrame = self.bridge.imgmsg_to_cv2(frameRgb, desired_encoding="passthrough")
        depthFrame = self.bridge.imgmsg_to_cv2(frameDepth, desired_encoding="passthrough")
        frame_distance = self.bridge.imgmsg_to_cv2(frameDepth, desired_encoding="32FC1")

        self.colorFrame = colorFrame.copy()
        self.depthFrame = depthFrame.copy()
        self.frame_distance = frame_distance.copy()

    def callbackOnlyRgb(self, frameRgb):
        colorFrame = self.bridge.imgmsg_to_cv2(frameRgb, desired_encoding="passthrough")
        self.colorFrame = colorFrame.copy()

    def setcameraInfo(self, cameraInfo, frame_id):
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = cameraInfo["width"]
        self.intrinsics.height = cameraInfo["height"]
        self.intrinsics.ppx = cameraInfo["K"][2]
        self.intrinsics.ppy = cameraInfo["K"][5]
        self.intrinsics.fx = cameraInfo["K"][0]
        self.intrinsics.fy = cameraInfo["K"][4]

        if cameraInfo["distortion_model"] == 'plumb_bob':
            self.intrinsics.model = rs2.distortion.brown_conrady
        elif cameraInfo["distortion_model"] == 'equidistant':
            self.intrinsics.model = rs2.distortion.kannala_brandt4
        self.intrinsics.coeffs = [i for i in cameraInfo["D"]]
        self.cameraInfoReceived = True

        # Reference frame
        self.frame_id = frame_id # "camera_color_optical_frame_id"

    def cameraInfoCallback(self, cameraInfo):
        """
        Callback for get Intrinsic Parameter of Camera and create intrinsics object (pyrealsense2 library)
        """
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = cameraInfo.width
        self.intrinsics.height = cameraInfo.height
        self.intrinsics.ppx = cameraInfo.K[2]
        self.intrinsics.ppy = cameraInfo.K[5]
        self.intrinsics.fx = cameraInfo.K[0]
        self.intrinsics.fy = cameraInfo.K[4]

        if cameraInfo.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs2.distortion.brown_conrady
        elif cameraInfo.distortion_model == 'equidistant':
            self.intrinsics.model = rs2.distortion.kannala_brandt4
        self.intrinsics.coeffs = [i for i in cameraInfo.D]
        self.cameraInfoReceived = True

        # Reference frame
        self.frame_id = cameraInfo.header.frame_id
        rospy.loginfo(f"Camera frame id: {self.frame_id}")

    def waitCameraInfo(self):
        while not self.cameraInfoReceived:
            pass
        self.sub_info.unregister()
        rospy.loginfo(PARAMETERS_LOG.format(self.intrinsics))

    def acquire(self):
        """
        Method for acquiring in syncronization way rgb and depth frame
        """
        self.subcriberColorFrame = message_filters.Subscriber(COLOR_FRAME_TOPIC, Image)
        self.subcriberDepthFrame = message_filters.Subscriber(DEPTH_ALIGNED_TOPIC, Image)
        # Subscriber Synchronization
        subSync = message_filters.TimeSynchronizer([self.subcriberColorFrame, self.subcriberDepthFrame], queue_size=10)
        # Call callback sincronized
        subSync.registerCallback(self.callback)

        rospy.spin()

    def acquireOnlyRgb(self):
        """
        Method for acquiring in syncronization way rgb
        """
        self.subcriberColor = rospy.Subscriber(COLOR_FRAME_TOPIC, Image, self.callbackOnlyRgb, queue_size=1)

    def acquireOnce(self):
        """Method for acquiring only once frame rgb
        """
        t0 = rospy.Time.now().to_sec()
        rospy.loginfo("Waiting frame ...")
        frameRgb = rospy.wait_for_message(COLOR_FRAME_TOPIC, Image, timeout=None)
        colorFrame = self.bridge.imgmsg_to_cv2(frameRgb, desired_encoding="bgr8")
        self.colorFrame = colorFrame.copy()
        rospy.loginfo("Frame recived...")
        rospy.loginfo(f"Elapsed time for acquisition: {rospy.Time.now().to_sec()-t0}")

    def acquireOnceBoth(self):
        """Method for acquiring only once frame rgb
        """
        rospy.loginfo("Waiting frame ...")
        frameRgb = rospy.wait_for_message(COLOR_FRAME_TOPIC, Image, timeout=None)
        # rospy.sleep(2.0)
        frameDepth = rospy.wait_for_message(DEPTH_ALIGNED_TOPIC, Image, timeout=None)

        colorFrame = self.bridge.imgmsg_to_cv2(frameRgb, desired_encoding="bgr8")
        self.colorFrame = colorFrame.copy()
        rospy.loginfo("Frame recived...")
        depthFrame = self.bridge.imgmsg_to_cv2(frameDepth, desired_encoding="passthrough")
        frame_distance = self.bridge.imgmsg_to_cv2(frameDepth, desired_encoding="32FC1")

        self.depthFrame = depthFrame.copy()
        self.frame_distance = frame_distance.copy()

    def getColorFrame(self):
        """Method for return frame acquired
        """
        return self.colorFrame

    def getDistanceFrame(self):
        """Method for return frame acquired (distance in mm)
        """
        return self.frame_distance

    def saveImage(self, folder_path):
        """Method for saving only one frame to  desired location

        Args:
            filename (String): path of the image to saved
        """

        self.acquireOnce()
        cv2.imwrite(folder_path + "frame_" + str(self.frame_number) + ".png",
                    self.colorFrame)  # ,cv2.cvtColor(self.colorFrame, cv2.COLOR_RGB2BGR))
        # cv2.imwrite(folder_path + "frame_" +str(self.frame_number) + ".png", self.colorFrame)
        self.frame_number += 1

    def saveAquiredImage(self, full_name):
        """Method for saving only one frame to  desired location

        Args:
            full_name (String): path with name of the image to saved
        """
        if self.colorFrame is not None:
            cv2.imwrite(full_name, self.colorFrame)

    def showColorFrame(self, nameWindowRgb):
        """Show RGB Frame in a windows

        Args:
            nameWindowRgb (String): Name of windows
        """
        imgImshow = cv2.cvtColor(self.colorFrame, cv2.COLOR_RGB2BGR)
        cv2.imshow(nameWindowRgb, imgImshow)

    def showImage(self, nameWindowRgb, nameWindowDepth):
        """
        Method for showing the image
        """
        # Rgb -> Bgr convertion for cv2 imshow
        imgImshow = cv2.cvtColor(self.colorFrame, cv2.COLOR_RGB2BGR)
        cv2.imshow(nameWindowRgb, imgImshow)
        cv2.imshow(nameWindowDepth, self.depthFrame)

    def getCameraParam(self):
        self.sub_info = rospy.Subscriber(CAMERA_INFO_TOPIC, CameraInfo, self.cameraInfoCallback)

    def stop(self):
        '''Method to disconnect the subscribers from kinect2_bridge topics, to release
            some memory and avoid filling up the queue.'''
        self.subcriberColorFrame.unregister()
        self.subcriberDepthFrame.unregister()

    def deproject(self, x, y, depth = None, update_frame = False) -> np.ndarray:
        """Deprojection: Image frame -> Camera frame (camera_color_optical_frame)

        Args:
            x (int): x-pixel. Column of the image "matrix"
            y (int): y-pixel. Row of the image matrix
            depth (float, optional): Depth in mm. If not provided, it will be retrieved from self.frame_distance.

        Returns:
            np.ndarray: [x, y, z] in camera frame (m)
        """
        if not self.cameraInfoReceived:
            print("Initialization not performed. Performing initialization.")
            self.initialize()
        if not self._valid_pixel(x,y):
            raise ValueError("Coordinates x and y must be integers & inside image.")
        if depth is None:
            if update_frame or self.frame_distance is None:
                self.acquireOnceBoth()            
            depth = self.frame_distance[y, x]
        deprojection = np.array(rs2.rs2_deproject_pixel_to_point(self.intrinsics, [x, y], depth)) / 1000.0
        return deprojection

    def _valid_pixel(self,x,y):
        if not isinstance(x, int) or not isinstance(y, int):
            print("Cordinates x and y must be integers")
            return False

        image_width = self.intrinsics.width
        image_height = self.intrinsics.height
        
        # Check that x and y are within the image dimensions
        if not (0 <= x < image_width) or not (0 <= y < image_height):
            print("Coordinates x and y must be within the image dimensions.")
            return False
        return True