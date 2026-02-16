from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uikit_web_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web_root'), glob('web_root/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ugv_dev',
    maintainer_email='pandey.himanshu27@google.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
