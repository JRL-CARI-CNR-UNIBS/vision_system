import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vision_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('scripts/*.py')),
        (os.path.join('share', package_name, 'test'), glob('test/test_bag/*'))
    ],
    install_requires=['setuptools',
                      'rosbags',
                      'pytest-dependency',
                      'numpy',
                      'opencv-python'],
    zip_safe=True,
    maintainer='Samuele Sandrini',
    maintainer_email='samuele.sandrini@polito.it',
    description='Vision System package for camera acquisition and post processing of rgb and depth images', 
    license='Apache Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'vision_system_node = scripts.vision_system_node:main',
        ],
    },
)
