from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share",package_name,"launch"),glob("launch/*")),
        (os.path.join("share",package_name,"worlds"),glob("worlds/*")),
        (os.path.join("share",package_name,"models"),glob("models/urdf/*")),
        (os.path.join('share', package_name, 'models', 'urdf'),glob('models/urdf/*.xacro')),
        (os.path.join("share",package_name,"models"),glob("models/meshes/*")),
        (os.path.join('share', package_name, 'models', 'meshes'),glob('models/meshes/*.stl')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yash',
    maintainer_email='yash@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
