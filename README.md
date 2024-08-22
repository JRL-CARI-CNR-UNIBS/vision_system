# 📷 Vision System Library

## Overview

This library is designed to facilitate the integration and management of cameras (acquisition side) within a ROS2 system. It provides a `Camera` class that simplifies the acquisition of frames from RGB and depth cameras and also supports custom processing functions via post-processing, and providing utilities.

## Features

- 🎥 **Frame Acquisition:** Support for acquiring color and depth frames from ROS2-compatible cameras. There are two methods for acquiring frames: using APIs (as one shot class methods) or starting the acquisition/processing phse (loop), the class is also a ROS2 launchable node.
- 🛠️ **Post-Processing:** The ability to apply a post-processing function to the acquired frames.
- 🔧 **ROS2 Integration:** Uses ROS2 topics and parameters to manage data.

