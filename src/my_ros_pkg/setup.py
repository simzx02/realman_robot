from setuptools import find_packages, setup

package_name = 'my_ros_pkg'

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
    maintainer='simzx',
    maintainer_email='xzhongs1127@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_circle = my_ros_pkg.robot_mover:main',
            'circular = my_ros_pkg.circular:main',
            'move_j = my_ros_pkg.move_j:main'
        ],
    },
)
