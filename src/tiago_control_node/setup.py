from setuptools import setup
import os
from glob import glob

package_name = 'tiago_control_node'

setup(
    name=package_name,
    version='0.5.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dionis Totsila',
    maintainer_email='dionis.totsila@inria.fr',
    description='Tiago OpenSoT Control Node',
    license='TODO',
    entry_points={
        'console_scripts': [
            'tiago_opensot_node = tiago_control_node.tiago_opensot_node:main',
            'cartesian_interface_node = tiago_control_node.cartesian_interface_node:main',
            'tiago_pro_opensot_node = tiago_control_node.tiago_pro_opensot_node:main',
            'dummy_obstacles = tiago_control_node.dummy_obstacles:main',
            'pose_commander = tiago_control_node.pose_commander:main',
        ],
    },
)