from glob import glob

from setuptools import find_packages, setup

package_name = 'drone_bringup'


setup(
    name=package_name,
    version='0.1.0',
    packages=[pkg for pkg in find_packages(exclude=['test']) if 'dashboard_static' not in pkg],
    package_data={
        package_name: [
            'dashboard_static/*',
            'dashboard_static/backgrounds/*',
            'dashboard_static/vendor/*',
            'dashboard_static/vendor/liquidglass/*',
        ],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name, ['PLANNERS.md', 'MAPS.md', 'SWARM.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='drone_ws',
    maintainer_email='student@local',
    description='Top-level launch files and YAML parameters for six acceptance scenarios.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'send_goal = drone_bringup.send_goal:main',
            'waypoint_publisher = drone_bringup.waypoint_publisher:main',
            'evaluate_drone = drone_bringup.evaluate:main',
            'formation_coordinator = drone_bringup.formation_coordinator:main',
            'ego_cmd_bridge = drone_bringup.ego_cmd_bridge:main',
            'local_sense_cloud = drone_bringup.local_sense_cloud:main',
            'inflate_vis_crop = drone_bringup.inflate_vis_crop:main',
            'mighty_cmd_bridge = drone_bringup.mighty_cmd_bridge:main',
            'pose_to_path_goal = drone_bringup.pose_to_path_goal:main',
            'cloud_bridge = drone_bringup.cloud_bridge:main',
            'map_adapter = drone_bringup.map_adapter_node:main',
            'dashboard = drone_bringup.dashboard_server:main',
            'drone_ws_console = drone_bringup.desktop_app:main',
            'interference_monitor = drone_bringup.interference_monitor:main',
        ],
    },
)