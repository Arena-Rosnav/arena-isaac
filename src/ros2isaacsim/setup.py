from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2isaacsim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brainfucker',
    maintainer_email='bigkatoan6969@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "run_isaacsim=ros2isaacsim.run_isaacsim:main",
            "convert_urdf_usd=ros2isaacsim.convert_urdf_usd:main",
            "control=ros2isaacsim.control:main",
        ],
    },
)
