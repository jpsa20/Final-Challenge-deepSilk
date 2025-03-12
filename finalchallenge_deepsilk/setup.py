from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'finalchallenge_deepsilk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
    ], 

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jp',
    maintainer_email='jp@todo.todo',
    description='Set point generator node for the final challenge',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'Input_deepsilk = finalchallenge_deepsilk.Input:main',  # Cambio aquí
        ],
    },
)