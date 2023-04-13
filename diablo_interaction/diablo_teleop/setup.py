from setuptools import setup

package_name = 'diablo_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vulcanyjx',
    maintainer_email='vulcanyjx@todo.com',
    description='Keyboard event capture',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "teleop_node = diablo_teleop.teleop:main"
        ],
    },
)

