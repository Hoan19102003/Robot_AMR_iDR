from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'main'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='.', include=['main', 'main.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hoan Pham',
    maintainer_email='hoan@example.com',
    description='Main control and GUI for AMR using ROS2 Humble',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main = main.main:main',
            'ROS2Publisher = main.Topic_python.cmd_vel:main',
            'odom_tf_broadcaster = main.Topic_python.odom_tf_broadcaster:main',
            'encoder_joint_state_publisher = main.Topic_python.joint_states:main',
            'fake_robot = main.fake_robot:main',  # <--- thêm dòng này
        ],
    },
)
