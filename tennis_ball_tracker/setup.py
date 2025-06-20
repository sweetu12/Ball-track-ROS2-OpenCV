from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tennis_ball_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sitanshu',
    maintainer_email='2003sitanshu@gmail.com',
    description='ROS 2 package for tennis ball detection using OpenCV.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_detector_node = tennis_ball_tracker.ball_detector_node:main',
        ],
    },
)
