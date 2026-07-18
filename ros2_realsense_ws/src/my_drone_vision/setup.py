from setuptools import find_packages, setup

package_name = 'my_drone_vision'

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
    maintainer='udeme',
    maintainer_email='udeme@todo.todo',
    description='Vision nodes for the custom PX4 Gazebo Classic quadcopter.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'realsense_square_detector = my_drone_vision.realsense_square_detector:main',
            'realsense_square_offboard_controller = my_drone_vision.realsense_square_offboard_controller:main',
        ],
    },
)
