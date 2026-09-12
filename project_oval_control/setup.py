from setuptools import setup

package_name = 'project_oval_control'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/config', [
        'config/oval_points.json',
        'config/pursuit_log.json',
    ]),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'numpy', 'opencv-python', 'pyvesc', 'crccheck'],
    zip_safe=True,
    maintainer='malinchc',
    maintainer_email='malinchc@gmail.com',
    description='Driving and actuation nodes for Project OVAL.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = project_oval_control.driver_node:main',
            'lane_follower = project_oval_control.lane_follower_node:main',
            'xbox_controller = project_oval_control.xbox_controller_node:main',
        ],
    },
)
