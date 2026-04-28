from setuptools import setup

package_name = 'agv_safety'

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
    description='AGV Safety package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'safety_arbiter_node = agv_safety.safety_arbiter_node:main',
            'emergency_stop_node = agv_safety.emergency_stop_node:main',
            'collision_monitor_node = agv_safety.collision_monitor_node:main',
            'system_watchdog_node = agv_safety.system_watchdog_node:main',
        ],
    },
)
