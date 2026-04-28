from setuptools import setup

package_name = 'agv_tour_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/tour_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Team',
    maintainer_email='agv@example.com',
    description='AGV Tour Manager package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tour_manager_node = agv_tour_manager.tour_manager_node:main',
            'audio_player_node = agv_tour_manager.audio_player_node:main',
            'passenger_display_node = agv_tour_manager.passenger_display_node:main',
        ],
    },
)
