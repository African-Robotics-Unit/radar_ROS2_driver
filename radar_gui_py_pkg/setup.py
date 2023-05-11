from setuptools import setup

package_name = 'radar_gui_py_pkg'
submodules = 'radar_gui_py_pkg/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nedwob',
    maintainer_email='nicholasbowden0@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radar_proc     =radar_gui_py_pkg.radar_proc_node:main',
            'radar_ctrl_gui =radar_gui_py_pkg.radar_client_node:main',
        ],
    },
)
