from setuptools import setup

package_name = 'robot_control'

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
    maintainer='root',
    maintainer_email='zahirmd1604@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = robot_control.teleop:main',
            'camera_view = robot_control.camera_view:main',
            'detect = robot_control.detect:main',
            'lidardata = robot_control.lidardata:main',
            'autonomous = robot_control.autonomous:main',
            'imudata = robot_control.imudata:main'
        ],
    },
)
