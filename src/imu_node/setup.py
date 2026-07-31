from setuptools import find_packages, setup

package_name = 'imu_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='IMU driver node: orientation, motion, fall/impact detection',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_driver_node = imu_node.imu_driver_node:main',
        ],
    },
)
