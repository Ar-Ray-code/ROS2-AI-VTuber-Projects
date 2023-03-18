from setuptools import setup

package_name = 'youtube_comment'

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
    description='YouTube comment reader',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'youtube_comment_node = youtube_comment.youtube_comment_node:main'
        ],
    },
)
