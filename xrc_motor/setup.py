from setuptools import find_packages, setup

package_name = 'xrc_motor'

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
    maintainer='Harald Haraldsson',
    maintainer_email='530725+haraldsson@users.noreply.github.com',
    description='ROS 2 package for motor control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "dynamixel_controller_node = xrc_motor.dynamixel_controller_node:main"
        ],
    },
)
