from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'image_conversion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='mahaveer',
    maintainer_email='2022uec1726@mnit.ac.in',
    description='Image conversion node with service-based mode switching between color and grayscale',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_conversion_node = image_conversion.image_conversion_node:main',
        ],
    },
)
