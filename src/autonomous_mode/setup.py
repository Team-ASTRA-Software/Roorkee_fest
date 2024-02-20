from setuptools import find_packages, setup

package_name = 'autonomous_mode'

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
    maintainer='anmol',
    maintainer_email='ec21b1086@iiitdm.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_logger=autonomous_mode.gps_logger:main',
            'key_input=autonomous_mode.key_input:main',
            'gps_logger_switch=autonomous_mode.gps_logger_switch:main'
        ],
    },
)
