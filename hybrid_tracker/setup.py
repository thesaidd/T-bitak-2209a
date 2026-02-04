from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hybrid_tracker'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdullah Gül University',
    maintainer_email='[email protected]',
    description='Hybrid visual-GPS tracking system for autonomous drone pursuit',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hybrid_tracker_node = hybrid_tracker.hybrid_tracker_node:main',
        ],
    },
)
