from setuptools import setup

package_name = 'agv_localization'

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
    description='AGV Localization package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ukf_node = agv_localization.ukf_node:main',
            'lidar_odometry_node = agv_localization.lidar_odometry_node:main',
            'gps_converter_node = agv_localization.gps_converter_node:main',
            'wheel_odometry_node = agv_localization.wheel_odometry_node:main',
        ],
    },
)
