from setuptools import setup
import os
from glob import glob

package_name = 'my_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Myungsang Lee',
    maintainer_email='mslee@robotegra.com',
    description='Vision AI 노드 (RF-DETR 검출 + 정렬 depth 거리) 및 카메라 뷰어',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detector_node = my_vision.detector_node:main',
            'camera_viewer = my_vision.camera_viewer:main',
        ],
    },
)
