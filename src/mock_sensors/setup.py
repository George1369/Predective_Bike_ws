from setuptools import find_packages, setup

package_name = 'mock_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mock_sensors.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Synthetic sensor publishers for PC-side development before hardware is wired up',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_imu_publisher = mock_sensors.mock_imu_publisher:main',
            'mock_gps_publisher = mock_sensors.mock_gps_publisher:main',
        ],
    },
)
