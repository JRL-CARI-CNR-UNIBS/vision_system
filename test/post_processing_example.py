
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

from vision_system.post_processing import PostProcessing
# All the import that you need
import cv2 as cv

class YOLOMock(PostProcessing):
  def process_frames(self, color_frame, distance_frame):
    print(f'Put post processing code here.')
    print(f'Example of post processing: {color_frame.shape}, {distance_frame.shape}')
  
  def process_frame(self, color_frame):
    pass  
  
class PostProcessingExample(PostProcessing):
  def process_frames(self, color_frame, distance_frame):
    # Show the color frame
    cv.imshow('Color Frame', color_frame)
    cv.waitKey(1)
    cv.destroyAllWindows()
  
  def process_frame(self, color_frame):
    pass  