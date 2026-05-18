from setuptools import find_packages, setup

package_name = 'cmd_vel_pub'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='t65599903@gmail.com',
    description='Publish TwistStamped to /cmd_vel topic.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_pub = cmd_vel_pub.cmd_vel_publisher:main',
        ],
    },
)
