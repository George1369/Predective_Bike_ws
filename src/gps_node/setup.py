from setuptools import find_packages, setup

package_name = 'gps_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='GPS/GNSS driver node: position, speed, route logging support',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gps_driver_node = gps_node.gps_driver_node:main',
        ],
    },
)
