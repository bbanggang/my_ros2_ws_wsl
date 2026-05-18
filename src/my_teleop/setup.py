from setuptools import find_packages, setup

package_name = 'my_teleop'

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
    description='Custom teleoperation node using keyboard for TurtleBot3.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = my_teleop.teleop_keyboard:main',
        ],
    },
)
