from setuptools import find_packages, setup

package_name = 'detection_visualizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    keywords=['ROS'],
    classifiers=[
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='Draws bounding boxes on an image from computer vision detections.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    package_data={
        package_name: ['dev_harness/messages/*.json'],
    },
    include_package_data=True,
    entry_points={
        'console_scripts': [
            'detection_visualizer = detection_visualizer:main',
            'detection_visualizer_harness = detection_visualizer.dev_harness.publisher:main',
            'detection_visualizer_snapshot = detection_visualizer.dev_harness.snapshot:main',
        ],
    },
)
