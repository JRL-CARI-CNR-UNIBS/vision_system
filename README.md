# vision_system

# Realsense Class

This Python class, named `Realsense`, is designed for subscribing to RealSense camera topics such as color and depth frames. It utilizes ROS (Robot Operating System) for communication and OpenCV for image processing. The class simplifies the acquisition of camera frames, camera parameter retrieval, and deprojection of image coordinates into 3D camera space.

## Usage

### 1. Installation

Install the required dependencies using the following command:

```bash
pip install -r requirements.txt
```

### 2. Class Initialization

```bash
realsense = Realsense()
realsense.initialize()
```

### 3. Deprojection
To deproject image pixels into 3D camera space (x,y,z) in m. There are two options. The first

```bash
point_in_camera_frame = realsense.deproject(x, y, depth)
point_in_camera_frame = realsense.deproject(x, y)

```
