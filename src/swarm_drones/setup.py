from setuptools import find_packages, setup

package_name = 'swarm_drones'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','pygame','pymavlink'],
    zip_safe=True,
    maintainer='hima',
    maintainer_email='hima@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_controller = swarm_drones.ROS2_Drone_Controller:main',
            'pymav_cmd_send = swarm_drones.Controller_cmd_send:main'
        ],
    },
)
