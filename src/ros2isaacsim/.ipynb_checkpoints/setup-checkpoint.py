from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2isaacsim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shibuina',
    maintainer_email='anhddhe180559@fpt.edu.vn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "run_isaacsim=ros2isaacsim.run_isaacsim:main",
            "convert_urdf_usd=ros2isaacsim.convert_urdf_usd:main",
            'navigation_controller = ros2isaacsim.navigation_controller:main',
            'sdf_to_urdf=ros2isaacsim.SdftoUrdf:main',
            ],
    },
)
