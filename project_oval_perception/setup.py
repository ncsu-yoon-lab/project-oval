from setuptools import find_packages, setup

package_name = 'project_oval_perception'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']) + [
        package_name + '.sidewalk_segmentation.sidewalk_segmentation_model',
    ],
    package_data={
        package_name: [
            'sidewalk_segmentation/*.json',
            'sidewalk_segmentation/sidewalk_segmentation_model/*',
        ],
    },
    include_package_data=True,
    data_files=data_files,
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'onnxruntime',
        'torch',
        'torchvision',
        'ultralytics',
        'matplotlib',
        'transformers',
        'safetensors',
        'pillow',
    ],
    zip_safe=True,
    maintainer='malinchc',
    maintainer_email='malinchc@gmail.com',
    description='Camera and ML perception nodes for Project OVAL.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'segmentation = project_oval_perception.segmentation_node:main',
            'obstacle_detector = project_oval_perception.obstacle_detector_node:main',
            'zed = project_oval_perception.zed_node:main',
            'yolo_od = project_oval_perception.yolo_od_node:main',
            'sidewalk_segmentation = project_oval_perception.sidewalk_segmentation.sidewalk_segmentation:main',
        ],
    },
)
