from setuptools import find_packages, setup
from glob import glob

package_name = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/urdf',   glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='GLaDOS@notanemail.com',
    description='Utilities for the PokerBot',
    license='None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grav_tune = utils.grav_tune:main',
            'traj_test = utils.traj_test:main'
        ],
    },
)
