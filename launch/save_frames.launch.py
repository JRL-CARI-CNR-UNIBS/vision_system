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

from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('vision_system')

    vision_config_path_cmd = DeclareLaunchArgument(
        'vision_config_path',
        default_value=join(pkg_dir, 'config', 'vision_system_config.yaml'),
        description='Full path to the vision system config file'
    )

    output_dir_cmd = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp/vision_system_dataset',
        description='Directory where acquired frames will be saved'
    )

    color_extension_cmd = DeclareLaunchArgument(
        'color_extension',
        default_value='.jpg',
        description='Color image extension: .jpg, .png, .jpeg, ...'
    )

    save_frames_node_cmd = Node(
        package='vision_system',
        executable='save_frames_node',
        name='save_frames_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('vision_config_path'),
            {
                'output_dir': LaunchConfiguration('output_dir'),
                'color_extension': LaunchConfiguration('color_extension'),
            }
        ]
    )

    ld = LaunchDescription()

    ld.add_action(vision_config_path_cmd)
    ld.add_action(output_dir_cmd)
    ld.add_action(color_extension_cmd)
    ld.add_action(save_frames_node_cmd)

    return ld