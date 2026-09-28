from setuptools import find_packages, setup

package_name = 'fpv_lab2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anna',
    maintainer_email='anna@todo.todo',
    description='Laboratory work 2 - ArduPilot ROS 2 flight test',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flight_test = fpv_lab2.flight_test.main:main',
            'point_flight = fpv_lab2.target_flight:main',
        ],
    },
)
