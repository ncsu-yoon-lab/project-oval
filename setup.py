from setuptools import setup


# ROS package name
package_name = 'project-oval'

# Python import package (had to change -, to _ so it could be importable by python)
python_package = 'project_oval'

# build data files array
data_files = []

# OG data files
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))

# Webots resources
data_files.append(('share/' + package_name + '/launch', ['launch/car_sim_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/test_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robot.urdf']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[python_package],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anglia',
    maintainer_email='anglia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = project_oval.sim_driver_node:main', 
            'pure_pursuit_node = project_oval.control.path_following.pure_pursuit.pure_pursuit_node:main',
            'gateway_node = project_oval.interface.gateway.gateway_node:main',
        ],
    },
)
