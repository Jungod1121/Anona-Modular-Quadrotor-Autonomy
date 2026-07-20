from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_rl_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/checkpoints', glob('checkpoints/*')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='drone_ws',
    maintainer_email='student@local',
    description='Local planners for drone_ws: VFH+, PPO, Polar DrQ-SAC (Path H).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vfh_planner_node = drone_rl_planner.vfh_planner_node:main',
            'rl_planner_node = drone_rl_planner.rl_planner_node:main',
            'sac_planner_node = drone_rl_planner.sac_planner_node:main',
            'safety_supervisor_node = drone_rl_planner.safety_supervisor_node:main',
            'train_ppo = drone_rl_planner.train_ppo:main',
            'train_mappo = drone_rl_planner.train_mappo:main',
            'train_sb3_ppo = drone_rl_planner.train_sb3_ppo:main',
            'train_sb3_mappo = drone_rl_planner.train_sb3_mappo:main',
            'train_sac_polar = drone_rl_planner.train_sac_polar:main',
        ],
    },
)
