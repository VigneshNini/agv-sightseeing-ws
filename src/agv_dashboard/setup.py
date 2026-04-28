from setuptools import setup

package_name = 'agv_dashboard'

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
    description='AGV Dashboard package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'web_bridge_node = agv_dashboard.web_bridge_node:main',
            'telemetry_logger_node = agv_dashboard.telemetry_logger_node:main',
        ],
    },
)
