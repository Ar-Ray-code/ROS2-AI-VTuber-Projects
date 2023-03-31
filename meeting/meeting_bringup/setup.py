from setuptools import setup

package_name = 'meeting_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/meet_nao.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ar-Ray-code',
    maintainer_email='ray255ar@gmail.com',
    description='AIVtuber bringup',
    license='Apache-2.0',
    tests_require=['pytest']
)
