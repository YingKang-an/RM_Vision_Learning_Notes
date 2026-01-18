from setuptools import setup

package_name = 'python_ros_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yinkangan',
    maintainer_email='kanganyin911@gmail.com',
    description='🎯 Python ROS2节点示例',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_node = python_ros_node.python_node:main',
        ],
    },
)
