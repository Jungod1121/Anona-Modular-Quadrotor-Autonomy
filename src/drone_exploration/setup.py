from setuptools import find_packages, setup

package_name = 'drone_exploration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/exploration.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drone_ws',
    maintainer_email='student@local',
    description='FUEL-style exploration layer for the shared drone plant.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'local_sensing = drone_exploration.local_sensing:main',
            'exploration_fsm = drone_exploration.exploration_fsm:main',
        ],
    },
)
