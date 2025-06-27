from setuptools import setup
import os
from glob import glob

package_name = 'xarm7_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azif',
    maintainer_email='azif@example.com',
    description='Bringup launch with planner and RViz for xArm7',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
