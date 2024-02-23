from setuptools import setup
from glob import glob

package_name = 'detectors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/card_features', glob('detectors/references/card_features/Back_of_Card.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Package demonstrating the camera and detector code',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'BackCardDetector  = detectors.BackCardDetector:main',
            # 'CardDetector  = detectors.CardDetector:main',
            # 'BackCardDetector  = detectors.BackCardDetector:main',
        ],
    },
)
