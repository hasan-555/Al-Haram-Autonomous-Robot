from setuptools import setup

package_name = 'crc_calculator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 node to calculate CRC-16 for a byte sequence',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crc_calculator = crc_calculator.crc_calculator:main',
            'decimal_to_hex_node = crc_calculator.decimal_to_hex_node:main',
            'send_modbus_series = crc_calculator.send_modbus_series:main',
            'request_data = crc_calculator.request_data:main',
        ],
    },
)
