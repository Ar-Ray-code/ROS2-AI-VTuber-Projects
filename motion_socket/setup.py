from setuptools import setup

package_name = 'motion_socket'

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
    maintainer='Ar-Ray-code',
    maintainer_email='ray255ar@gmail.com',
    description='Motion socket node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_socket_node = motion_socket.motion_socket_node:main'
        ],
    },
)
