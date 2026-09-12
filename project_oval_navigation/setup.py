import os
from glob import glob

from setuptools import setup

package_name = 'project_oval_navigation'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'maps'), glob('maps/*.csv')),
    (os.path.join('share', package_name, 'routes'), glob('routes/*.csv')),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.web_ui_static'],
    package_data={
        package_name: ['web_ui_static/*'],
    },
    include_package_data=True,
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='malinchc',
    maintainer_email='malinchc@gmail.com',
    description='Graph-based global navigation for Project OVAL.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner_node = project_oval_navigation.global_planner_node:main',
            'global_set_reference = project_oval_navigation.global_set_reference:main',
            'pure_pursuit_node = project_oval_navigation.pure_pursuit_node:main',
            'oval_web_ui = project_oval_navigation.oval_web_ui:main',
            'frame_projection = project_oval_navigation.frame_projection:main',
        ],
    },
)
