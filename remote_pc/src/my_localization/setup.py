import os
from glob import glob

from setuptools import setup

package_name = 'my_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Myungsang Lee',
    maintainer_email='mslee@robotegra.com',
    description='자동 위치 찾기 (전역 위치 추정): 라이다 스캔을 저장 지도에 맞춰 AMCL 초기 위치를 만든다',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'relocalizer_node = my_localization.relocalizer_node:main',
        ],
    },
)
