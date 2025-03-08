from setuptools import setup, find_packages
import glob
import os

package_name = 'wall_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', "wall_follower/params.yaml"]),
        ('share/wall_follower/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
        ('share/wall_follower/launch', glob.glob(os.path.join('launch', '*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "example=wall_follower.example:main",
            "safety_example=wall_follower.safety:main",
            "wall_follower = wall_follower.wall_follower:main",
            'test_wall_follower = wall_follower.test_wall_follower:main',
        ],
    },
)
