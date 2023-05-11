from setuptools import setup
import os
from glob import glob

package_name = 'radar_dca1000_py_pkg'
submodules = 'radar_dca1000_py_pkg/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/', package_name), glob('launch/radar_dca1000_launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='lindelanimcebo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radar_ctrl   =radar_dca1000_py_pkg.radar_ctrl_node:main',
            'radar_clnt   =radar_dca1000_py_pkg.radar_clnt_node:main',
            'radar_data   =radar_dca1000_py_pkg.radar_data_node:main',
        ],
    },
)
