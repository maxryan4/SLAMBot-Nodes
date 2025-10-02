from setuptools import find_packages, setup

package_name = 'motor_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motor_launch.py']),
    ],
    install_requires=['setuptools', 'rclpy'],
    maintainer='TODO',
    maintainer_email='TODO',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
             'motor_node = motor_node.motor_node:main',
        ],
    },
)