
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


from abc import ABC, abstractmethod

class PostProcessing(ABC):
  def __init__(self):
    pass

  def initialize(self, camera_info):
    pass
  
  @abstractmethod
  def process_frames(self, color_frame, distance_frame):
    pass
  
  @abstractmethod
  def process_frame(self, color_frame): # Can be implemented as empty if not needed
    pass  
