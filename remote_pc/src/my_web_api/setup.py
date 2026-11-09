from setuptools import setup
import os
from glob import glob

package_name = 'my_web_api'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Myungsang Lee',
    maintainer_email='mslee@robotegra.com',
    description='웹 UI 백엔드 (FastAPI + rclpy): 로봇 상태·카메라 MJPEG API',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = my_web_api.server:main',
        ],
    },
)
