from setuptools import find_packages, setup

package_name = 'ag_laserscan_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='toughbook',
    maintainer_email='Ryanshiau1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ag_laserscan_sub = ag_laserscan_package.ag_laser_scan:main',
            'ag_linedetection_node = ag_laserscan_package.ag_line_detection:main',
            'ag_laser_dist_orien_calc = ag_laserscan_package.ag_laser_dist_orien_calculation:main',
            'alter_laser_scan = ag_laserscan_package.alter_laser_scan:main'
        ],
    },
)
