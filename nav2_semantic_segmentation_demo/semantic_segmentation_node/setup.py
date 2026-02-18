from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'semantic_segmentation_node'

setup(
    name='semantic_segmentation_node',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.onnx')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='pedro.gonzalez@eia.edu.co',
    description='ROS2 node for semantic segmentation inference',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'segmentation_node = semantic_segmentation_node.segmentation_node:main',
        ],
    },
)
