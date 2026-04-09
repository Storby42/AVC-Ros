from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'waypointstarter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name,'config'), glob('config/*.yaml')),
	(os.path.join('share', package_name, 'data'), glob('data/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='storby',
    maintainer_email='70710230+Storby42@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'waypointstarter=waypointstarter.waypointstarter:main'
        ],
    },
)
