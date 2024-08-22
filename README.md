# 📷 Vision System Library

## Overview

This library is designed to facilitate the integration and management of cameras (acquisition side) within a ROS2 system. It provides a `Camera` class that simplifies the acquisition of frames from RGB and depth cameras and also supports custom processing functions via post-processing, and providing utilities.

## Features

- 🎥 **Frame Acquisition:** Support for acquiring color and depth frames from ROS2-compatible cameras. There are two methods for acquiring frames: using APIs (as one shot class methods) or starting the acquisition/processing phse (loop), the class is also a ROS2 launchable node.
- 🛠️ **Post-Processing:** The ability to apply a post-processing function to the acquired frames.
- 🔧 **ROS2 Integration:** Uses ROS2 topics and parameters to manage data.

### Installation

To install the library, clone the repository and build it using colcon (inside src folder of your workspace):

```bash
git clone https://github.com/your-repo/vision_system.git
cd vision_system
pip install -r requirements.txt
cd ..
colcon build --symlink-install
```

## Usage

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](https://github.com/SamueleSandrini/vision_system/blob/main/LICENSE) file for details.