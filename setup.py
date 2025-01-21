from setuptools import find_packages, setup

package_name = 'canopen_manager_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['canopen_motor_module/config/ZeroErr Driver_V1.5.eds']),
    ],
    install_requires=['setuptools', 'canopen', 'serial', 'time', 'math', 'random'],
    zip_safe=True,
    maintainer='shs',
    maintainer_email='minimirror1@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'canopen_manager_node = canopen_manager_pkg.canopen_manager_node:main',
        ],
    },
)
