from setuptools import setup
import os

package_name = 'ti_es_gps_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'adafruit-circuitpython-gps'],
    zip_safe=True,
    maintainer='luxo',
    maintainer_email='luxo@example.com',
    description='ROS2 package for GPS module',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ti_es_gps_node = ti_es_gps_package.ti_es_gps_node:main'
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['ti_es_gps_package/setup_permissions.py'])
    ]
)
