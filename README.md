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
git clone https://github.com/SamueleSandrini/vision_system
cd vision_system
pip install -r requirements.txt
cd ..
colcon build --symlink-install
```

## Usage

### Camera Node Class
The Camera class serves as a ROS2 node and can be used both directly as a regular class with its [API](https://jrl-cari-cnr-unibs.github.io/vision_system/) methods or run as a ROS2 node. Below is a summary of the main API methods available in the class:

It follows a table summarizing the main class API method.

| Method                          | Description                                                                                                           |
|---------------------------------|-----------------------------------------------------------------------------------------------------------------------|
| `retrieve_camera_info`          | Retrieves the camera information and stores it in the class.                                                           |
| `acquire_color_frame_once`      | Acquires a single color frame.                                                                                        |
| `acquire_frames_once`           | Acquires a single pair of color and depth frames.                                                                      |
| `process_once`                  | Processes a single pair of color and depth frames using the provided post-processing function.                         |
| `set_processing_function`       | Dynamically loads and sets the post-processing function.                                                                |
| `start_acquire`                 | Starts the synchronized acquisition of color and depth frames, with post-processing.                                   |
| `start_acquire_only_color`      | Starts the acquisition of color frames only, with post-processing.                                                      |
| `get_frames`                    | Retrieves the most recently acquired color and depth frames.                                                            |
| `get_color_frame`               | Retrieves the most recently acquired color frame.                                                                      |
| `get_distance_frame`            | Retrieves the most recently acquired depth frame.                                                                       |
| `get_camera_info`               | Retrieves the camera information.                                                                                      |
| `get_frame_id`                  | Retrieves the frame ID from the camera information.                                                                    |

### PostProcessing Class

Additionally, you can configure a custom post-processing function for the acquisition process. This is useful to avoid rewriting the same code repeatedly by simply adding the custom function to the pipeline. Examples of post-processing functions include object detection with YOLO, broadcasting transforms, displaying images with `cv2.imshow`, etc. To do this, you need to create a `CustomPostProcessing` class that inherits from [PostProcessing](https://github.com/JRL-CARI-CNR-UNIBS/vision_system/blob/main/vision_system/post_processing.py), which is an abstract class with two methods that must be implemented:

- `process_frames(self, color_frame, distance_frame)`: This method is automatically called inside `process_once` or `start_acquire`.
- `process_frame(self, color_frame)`: This method is used only when `start_acquire_only_color` is employed.

If any of these methods are not required, you can implement them as empty methods (using `pass`).

### Utilities

The module [`vision_system_utils.py`](https://github.com/JRL-CARI-CNR-UNIBS/vision_system/blob/main/vision_system/vision_system_utils.py) provides several utilities, such as methods to convert pixel coordinates and depth into corresponding 3D points.


### Examples

You can find an example of usage of Camera Node and of PostPorcessing definition here: [`Examples`](https://github.com/JRL-CARI-CNR-UNIBS/vision_system/tree/main/examples).


### If you want to use the already defined node, use the config file to set the topic name and others config, and use the launch file:

```bash
ros2 launch vision_system vision_system.launch.py
```

## Contribution

Each contribution, especially about utilities, is really welcome ;).

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](https://github.com/JRL-CARI-CNR-UNIBS/vision_system/blob/main/LICENSE) file for details.
