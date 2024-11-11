from setuptools import setup
from glob import glob  # Import glob here
import os

package_name = 'mujoco_ros_mobile'

# Collect all files in the robots directory recursively
data_files = [
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
]

# Traverse the robots directory recursively and add all files
for root, dirs, files in os.walk('robots'):
    for file in files:
        # Append each file with its relative path to data_files
        data_files.append((os.path.join('share', package_name, root), [os.path.join(root, file)]))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Your package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = mujoco_ros_mobile.robot_controller:main'
        ],
    },
)
