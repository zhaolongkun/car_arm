import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'rebot_gas_spray_demo'


def package_files(directory):
    return [
        path for path in glob(os.path.join(directory, '*'))
        if os.path.isfile(path)
    ]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), package_files('launch')),
        (os.path.join('share', package_name, 'config'), package_files('config')),
        (os.path.join('share', package_name, 'rviz'), package_files('rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='version-car',
    maintainer_email='user@example.com',
    description='Standalone reBot B601-DM gas leak spray simulation demo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gas_field_simulator = rebot_gas_spray_demo.gas_field_simulator:main',
            'spray_simulator = rebot_gas_spray_demo.spray_simulator:main',
            'rebot_spray_task = rebot_gas_spray_demo.rebot_spray_task:main',
            'rebot_circle_point_task = rebot_gas_spray_demo.rebot_circle_point_task:main',
        ],
    },
)
