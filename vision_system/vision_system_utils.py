
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


import cv2
from sensor_msgs.msg import CameraInfo
import numpy as np

def deproject_pixel_to_point(self, pixel, distance, camera_info: CameraInfo):
  """
  Deproject a pixel to a 3D point
  :param pixel: (u, v) pixel coordinates
  :param depth: depth value
  :return: (x, y, z) 3D point
  """
  if self.camera_info is None:
      return None
  
  # Camera intrinsics
  fx = self.camera_info.K[0]
  fy = self.camera_info.K[4]
  ppx = self.camera_info.K[2]
  ppy = self.camera_info.K[5]
  coeffs = [coeff for coeff in self.camera_info.D]

  u = pixel[0]
  v = pixel[1]
  x = (u - ppx) / fx
  y = (v - ppy) / fy
  camera_matrix = np.array([[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]])
  distortion_coefficients = np.array(coeffs)

  undistorted_point = cv2.undistortPoints(np.array([[u, v]], dtype=np.float32), camera_matrix, distortion_coefficients)
  x, y = undistorted_point[0][0]

  return (distance * x, distance * y, distance)
