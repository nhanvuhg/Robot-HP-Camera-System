import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fill_hp_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'qml'), glob(package_name + '/qml/*.qml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='vjetstore.com@gmail.com',
    description='Standalone QML preview for Fill HP Control.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'fill_hp_gui = fill_hp_gui.main:main',
        ],
    },
)
