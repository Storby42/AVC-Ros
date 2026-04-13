from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'buckalization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'data'), glob('data/*.csv')),
        (os.path.join('share', package_name, 'data'), glob('data/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Annabeth Pan',
    maintainer_email='annabethpan7@gmail.com',
    description='Creates a more accurate pose estimate from vision and other odom estimation',
    license='Apache-2.0',
    
    entry_points={
        'console_scripts': [
            'buckalization=buckalization.buckalization:main',
            'buckalizer_LSD=buckalization.buckalizer_LSD:main'
        ],
    },
)
