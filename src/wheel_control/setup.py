from setuptools import find_packages, setup

package_name = 'wheel_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','pyserial','rclpy'],
    zip_safe=True,
    maintainer='orl1',
    maintainer_email='dmj17b@fsu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_control = wheel_control.wheel_control:main',
        ],
    },
)
