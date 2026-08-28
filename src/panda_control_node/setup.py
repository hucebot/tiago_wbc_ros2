from setuptools import setup
import os
from glob import glob

package_name = 'panda_control_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.tasks'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Constantinos Tsakonas',
    maintainer_email='iamtsac@gmail.com',
    description='Minimal single-arm OpenSoT control node for Franka Panda - a WBC ablation against tiago_control_node',
    license='TODO',
    entry_points={
        'console_scripts': [
            'panda_opensot_node = panda_control_node.panda_opensot_node:main',
            'episode_manager = panda_control_node.episode_manager:main',
        ],
    },
)
