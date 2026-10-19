from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'adaptive_episodic_memory'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DSC',
    maintainer_email='cjyt.69420@gmail.com',
    description='Adaptive Episodic Memory System for Autonomous Robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'memory_manager = adaptive_episodic_memory.memory.memory_manager_node:main',
            'landmark_manager = adaptive_episodic_memory.memory.landmark_manager_node:main',
            'route_replayer = adaptive_episodic_memory.memory.route_replayer_node:main',
            'yolo_perception = adaptive_episodic_memory.perception.yolo_perception_node:main',
            'twist_relay = adaptive_episodic_memory.utils.twist_relay:main',
            'perception_bridge = adaptive_episodic_memory.perception_bridge_node:main',
            'auto_explorer = adaptive_episodic_memory.utils.auto_explorer:main',
            'waypoint_sweeper = adaptive_episodic_memory.utils.waypoint_sweeper:main',
            'teleop_wasd = adaptive_episodic_memory.utils.teleop_wasd:main',
            'go_to_memory = adaptive_episodic_memory.utils.go_to_memory:main',
            'yolo_probe = adaptive_episodic_memory.utils.yolo_probe:main',
            'dashboard_node = adaptive_episodic_memory.reasoning.dashboard_node:main',
            'web_bridge = adaptive_episodic_memory.utils.aems_web_bridge:main',
        ],
    },    
        
    
)
