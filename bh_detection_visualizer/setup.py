from setuptools import setup

package_name = 'bh_detection_visualizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=['bh_detection_visualizer'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Lewis Riches',
    author_email='lewis.riches99@gmail.com',
    maintainer='Lewis Riches',
    maintainer_email='lewis.riches99@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='Draws bounding boxes on an image from computer vision detections.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bh_detection_visualizer = bh_detection_visualizer:main',
        ],
    },
)
