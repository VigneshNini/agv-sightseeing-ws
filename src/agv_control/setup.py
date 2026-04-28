from setuptools import setup

package_name = 'agv_control'

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
    description='AGV Control package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lqr_controller = agv_control.lqr_controller:main',
            'can_bus_interface = agv_control.can_bus_interface:main',
            'motor_controller_node = agv_control.motor_controller_node:main',
            'steering_controller = agv_control.steering_controller:main',
        ],
    },
)
