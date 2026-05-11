from setuptools import find_packages, setup

package_name = 'rclpy_pkg1'

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
    maintainer='linux',
    maintainer_email='hungue6559@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pub = rclpy_pkg1.pub:main',
            'sub = rclpy_pkg1.sub:main',
            'twist_pub = rclpy_pkg1.twist_pub:main',
        ],
    },
)
