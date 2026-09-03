from glob import glob
from setuptools import find_packages, setup

package_name = 'graph_nav_with_pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['web_ui_static/*'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tkalra',
    maintainer_email='tkalra@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'driver = graph_nav_with_pure_pursuit.driver_node:main',
            'pure_pursuit_node = graph_nav_with_pure_pursuit.pure_pursuit_node:main',
            'xbox_controller = graph_nav_with_pure_pursuit.xbox_controller_node:main',
            'global_set_reference = graph_nav_with_pure_pursuit.global_set_reference:main',
            'global_planner_node = graph_nav_with_pure_pursuit.global_planner_node:main',
            'oval_web_ui = graph_nav_with_pure_pursuit.oval_web_ui:main',
        ],
    },
)
