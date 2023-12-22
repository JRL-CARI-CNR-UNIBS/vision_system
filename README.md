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
To deproject image pixels into 3D camera space (x,y,z) in m. There are two options. There are two Options for performing deprojection:

#### Option 1: Deprojection with Given Depth
```bash
point_in_camera_frame = realsense.deproject(x, y, depth)
```
This option allows you to provide the x and y pixel coordinates on the image along with a specific depth value. The method then deprojects these coordinates into 3D camera space, considering the provided depth. The result, point_in_camera_frame, is a NumPy array representing the [x, y, z] coordinates in the camera frame, where x, y, and z are in meters.

#### Option 2: Deprojection with Automatic Depth Retrieval
```bash
point_in_camera_frame = realsense.deproject(x, y)
```
In this option, you provide only the x and y pixel coordinates on the image, and the method automatically retrieves the depth value from the most recent depth frame acquired by the camera. If the depth frame is not available, it triggers the acquisition of both color and depth frames synchronously. After retrieving the depth, the deprojection is performed as in Option 1.

Note:

The automatic depth retrieval option is useful when you want to deproject image coordinates into 3D space without explicitly providing the depth value. It simplifies the process by internally handling the acquisition of the required frames.
The manual depth option is useful when you have a specific depth value that you want to use for deprojection, and you provide this depth as an argument to the method.
Choose the option that best suits your use case based on whether you have a specific depth value or prefer automatic retrieval from the most recent depth frame.






