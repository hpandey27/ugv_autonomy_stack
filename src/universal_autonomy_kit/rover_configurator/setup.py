from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_configurator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config/templates'), glob('config/templates/*.xacro')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='ugv_dev',
    maintainer_email='pandey.himanshu27@google.com',
    description='Universal Rover Configurator',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_urdf = rover_configurator.description_generator:main',
        ],
    },
)
