from setuptools import setup

package_name = 'agv_planning'

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
    description='AGV Planning package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'global_route_manager = agv_planning.global_route_manager:main',
            'hybrid_astar_planner = agv_planning.hybrid_astar_planner:main',
            'mpc_local_planner = agv_planning.mpc_local_planner:main',
            'velocity_profiler = agv_planning.velocity_profiler:main',
            'behavior_tree_node = agv_planning.behavior_tree_node:main',
        ],
    },
)
