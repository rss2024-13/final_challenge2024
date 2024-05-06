import os
import glob
from setuptools import setup

package_name = 'final_challenge2024'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/final_challenge2024/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='jianmc@mit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parking_controller = final_challenge2024.parking_controller:main',
            'lane_detector = final_challenge2024.lane_detector:main',
            'homography_transformer = final_challenge2024.homography_transformer:main',
        ],
    },
)
