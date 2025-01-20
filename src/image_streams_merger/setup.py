from setuptools import find_packages, setup

package_name = 'image_streams_merger'

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
    maintainer='Tim Yan Muk',
    maintainer_email='tcyanmuk@gmail.com',
    description='Package used to merge camera streams to a single panoramic view',
    license='N/A',
    entry_points={
        'console_scripts': [
            'image_streams_merger = image_streams_merger.main:main',
        ],
    },
)
