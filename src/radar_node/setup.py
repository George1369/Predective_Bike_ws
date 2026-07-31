from setuptools import find_packages, setup

package_name = 'radar_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Radar driver node: range sensing for obstacle detection',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'radar_driver_node = radar_node.radar_driver_node:main',
        ],
    },
)
