
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

#!/usr/bin/env python3

from pathlib import Path
from datetime import datetime

import cv2
import numpy as np
import rclpy

from vision_system.camera import Camera
from vision_system.vision_system_utils import save_color_image, save_depth_npy

class SaveFramesNode(Camera):

    def __init__(self):
        super().__init__()

        self.declare_parameter("output_dir", "/tmp/vision_system_dataset")
        self.declare_parameter("color_extension", ".jpg")

        self.output_dir = Path(
            self.get_parameter("output_dir")
            .get_parameter_value()
            .string_value
        )

        self.color_extension = (
            self.get_parameter("color_extension")
            .get_parameter_value()
            .string_value
        )

        if not self.color_extension.startswith("."):
            self.color_extension = f".{self.color_extension}"

        self.color_dir = self.output_dir / "color"
        self.depth_dir = self.output_dir / "depth"

        self.counter = 0

        self.get_logger().info(f"Output directory: {self.output_dir}")
        self.get_logger().info(f"Color extension: {self.color_extension}")

    def save_next_frame(self, save_depth: bool) -> bool:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"frame_{self.counter:06d}_{timestamp}"

        if save_depth:
            color_frame, depth_frame = self.acquire_frames_once()

            if color_frame is None or depth_frame is None:
                self.get_logger().error("Failed to acquire color/depth frames.")
                return False

            color_path = self.color_dir / f"{filename}{self.color_extension}"
            depth_path = self.depth_dir / f"{filename}.npy"

            color_saved = save_color_image(color_frame, color_path)
            depth_saved = save_depth_npy(depth_frame, depth_path)

            if not color_saved:
                self.get_logger().error(f"Failed to save color image: {color_path}")
                return False

            if not depth_saved:
                self.get_logger().error(f"Failed to save depth frame: {depth_path}")
                return False

            self.get_logger().info(f"Saved color: {color_path}")
            self.get_logger().info(f"Saved depth: {depth_path}")

        else:
            color_frame = self.acquire_color_frame_once()

            if color_frame is None:
                self.get_logger().error("Failed to acquire color frame.")
                return False

            color_path = self.color_dir / f"{filename}{self.color_extension}"

            color_saved = save_color_image(color_frame, color_path)

            if not color_saved:
                self.get_logger().error(f"Failed to save color image: {color_path}")
                return False

            self.get_logger().info(f"Saved color: {color_path}")

        self.counter += 1
        return True


def ask_save_depth() -> bool:
    answer = input("Save depth too? [y/N]: ").strip().lower()
    return answer in ["y", "yes"]


def main(args=None):
    rclpy.init(args=args)

    node = SaveFramesNode()

    try:
        save_depth = ask_save_depth()

        node.get_logger().info(
            "Press ENTER to save a frame, or type 'q' and press ENTER to quit."
        )

        while rclpy.ok():
            command = input("> ").strip().lower()

            if command in ["q", "quit", "exit"]:
                break

            node.save_next_frame(save_depth)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()