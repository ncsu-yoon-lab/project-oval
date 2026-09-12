import os
from glob import glob

from setuptools import setup

package_name = 'project_oval_bringup'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='malinchc',
    maintainer_email='malinchc@gmail.com',
    description='Launch files, URDF, and worlds that bring up Project OVAL.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
