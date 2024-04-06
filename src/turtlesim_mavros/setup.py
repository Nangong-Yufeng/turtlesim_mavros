from setuptools import find_packages, setup
from glob import glob

package_name = 'turtlesim_mavros'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('images', glob('images/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moxljk',
    maintainer_email='moxljk@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlesim = turtlesim_mavros.turtlesim:main'
        ],
    },
)
