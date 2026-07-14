from glob import glob

from setuptools import find_packages, setup

package_name = 'drone_bringup'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            'dashboard_static/*',
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
    zip_safe=True,
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
            'cloud_bridge = drone_bringup.cloud_bridge:main',
            'dashboard = drone_bringup.dashboard_server:main',
        ],
    },
)