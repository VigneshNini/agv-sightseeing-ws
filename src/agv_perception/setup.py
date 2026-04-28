from setuptools import setup

package_name = 'agv_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Team',
    maintainer_email='agv@example.com',
    description='AGV Perception package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lidar_obstacle_detector = agv_perception.lidar_obstacle_detector:main',
            'object_tracker_node = agv_perception.object_tracker_node:main',
            'camera_perception_node = agv_perception.camera_perception_node:main',
            'driveable_surface_node = agv_perception.driveable_surface_node:main',
        ],
    },
)
