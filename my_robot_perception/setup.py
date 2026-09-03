import os
from setuptools import setup

package_name = 'my_robot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cxl',
    maintainer_email='changxianli6121@163.com',
    description='ArUco object detection (M1)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aruco_detector = my_robot_perception.aruco_detector:main',
        ],
    },
)
