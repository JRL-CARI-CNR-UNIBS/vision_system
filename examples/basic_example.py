from vision_system.camera import Camera
import vision_system.vision_system_utils as vision_utils
import rclpy
import cv2 as cv

def main():
    rclpy.init()
    camera = Camera(
        color_image_topic = '/head_front_camera/rgb/image_raw',
        depth_image_topic = '/head_front_camera/depth_registered/image_raw',
        camera_info_topic = '/head_front_camera/depth_registered/camera_info'
    )
    if not camera.retrieve_camera_info():
        return
    # If necessary, set the post processing function
    # camera.set_processing_function(package_name='examples', 
    #                                module_name='post_processing_example', 
    #                                class_name='PostProcessingExample')
    
    rgb_frame, distance_frame  = camera.acquire_frames_once()
    # Do whatever you want with the frames example show them  
    camera_info = camera.get_camera_info()

    print(vision_utils.deproject_pixel_to_point([100, 150], distance_frame[100,150], camera_info))

    cv.imshow('Color Frame', rgb_frame)
    cv.imshow('Distance Frame', distance_frame)
    cv.waitKey(0)
    cv.destroyAllWindows()



    # Otherwise if you want to use the processing functio and so do nothing here call:
    # camera.process_once() instead of camera.acquire_frames_once

    camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
