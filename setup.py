from setuptools import setup, find_packages

package_name = 'project-oval'
python_package = 'project_oval'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/launch', ['launch/car_sim_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/test_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robot.urdf']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),   # ← discovers project_oval + all subpackages
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