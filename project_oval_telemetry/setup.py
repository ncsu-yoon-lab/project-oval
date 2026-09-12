from setuptools import setup

package_name = 'project_oval_telemetry'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'folium',
        'eventlet',
        'pyserial',
        'google-cloud-pubsub',
    ],
    zip_safe=True,
    maintainer='malinchc',
    maintainer_email='malinchc@gmail.com',
    description='Sensor and telemetry communication nodes for Project OVAL.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu = project_oval_telemetry.imu_node:main',
            'fake_rtk = project_oval_telemetry.fake_rtk:main',
            'zed_heading = project_oval_telemetry.zed_heading:main',
            'oval_message = project_oval_telemetry.oval_message:main',
            'telemetry_ros2 = project_oval_telemetry.telemetry_ros2:main',
            'pub_sub_google = project_oval_telemetry.pub_sub_google:main',
            'map_test = project_oval_telemetry.map_test:main',
            'message_display = project_oval_telemetry.message_display:main',
        ],
    },
)
