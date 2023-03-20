from setuptools import setup
from glob import glob
import os

package_name = 'chatgpt_socket'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('./config/*'))
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
            'chatgpt_socket_node = chatgpt_socket.chatgpt_socket_node:main'
        ],
    },
)
