from setuptools import find_packages, setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khushi',
    maintainer_email='khushi@todo.todo',
    description='ROS2-based robot controller with manual and autonomous drive modes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             # Main robot mode manager (switches between manual and auto)
          'main_node = robot_controller.main:main',

        # RC control node (manual driving)
        'rccontrol_node = robot_controller.rccontrol:main',

        # Autonomous drive node
        'autodrive_node = robot_controller.autodrive:main',
        ],
    },
)



