from setuptools import find_packages, setup

package_name = 'my_robot'

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
    maintainer='bot-basimuz',
    maintainer_email='basimuzbot@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'hardware_node = my_robot.hardware.hardware_node:main',
        'room_explorer = my_robot.behaviors.room_explorer_node:main',
        'servo_controller = my_robot.hardware.servo_controller:main',
    ],
  },
)

