from setuptools import find_packages, setup
from glob import glob

package_name = 'hamster_line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sm',
    maintainer_email='hcooch2ch3@gmail.com',
    description='Camera-based line following package for Hamster robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower_node = hamster_line_follower.line_follower_node:main',
        ],
    },
)
