from setuptools import setup

package_name = 'agv_mapping'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/ndt_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Team',
    maintainer_email='agv@example.com',
    description='AGV Mapping package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ndt_mapping_node = agv_mapping.ndt_mapping_node:main',
            'octomap_node = agv_mapping.octomap_node:main',
            'map_server_node = agv_mapping.map_server_node:main',
        ],
    },
)
