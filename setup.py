from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'g1pilot'

def expand(patterns):
    files = []
    for p in patterns:
        files.extend(glob(p, recursive=True))
    return files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),

        # Launch Files
        (f'share/{package_name}/launch', [
            'launch/navigation_demo.launch.py',
            'launch/navigation_walk.launch.py',
            'launch/navigation_mapping.launch.py',
            'launch/navigation_hardware.launch.py',
        ]),

        # URDF / XML
        (f'share/{package_name}/description_files/urdf',
         ['description_files/urdf/g1_29dof.urdf']),

        # Meshes
        (f'share/{package_name}/description_files/meshes',
         expand([
            'description_files/meshes/**/*.STL',
         ])),

        # Configuration Files
        (f'share/{package_name}/config', [
            'config/slam_toolbox.yaml',
            'config/slam_toolbox_hardware.yaml',
        ]),

        # RViz
        (f'share/{package_name}/rviz',
         ['config/navigation_mapping.rviz']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clemente Donoso',
    maintainer_email='clemente.donoso@inria.fr',
    description='MuJoCo and ROS 2 navigation simulation for Unitree G1',
    license='BSD 3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Navigation Nodes
            'dijkstra_planner = g1pilot.navigation.dijkstra_planner:main',
            'nav2point = g1pilot.navigation.nav2point:main',
            'create_map = g1pilot.navigation.create_map:main',
            'navigation_sim = g1pilot.navigation.navigation_sim:main',
            'g1_hardware_bridge = g1pilot.navigation.g1_hardware_bridge:main',
            'g1_sensor_bridge = g1pilot.navigation.g1_sensor_bridge:main',
        ],
    },
)
