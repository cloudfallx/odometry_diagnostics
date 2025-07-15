from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pub_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # The following line installs all launch/.py files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'matplotlib',
        # Add other dependencies as needed (e.g., tf2_ros)
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='prasadithya24@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "odom_plotter = pub_sub.odom_plotter:main"
        ],
    },
)
