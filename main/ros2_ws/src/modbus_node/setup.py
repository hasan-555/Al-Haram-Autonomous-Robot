from setuptools import find_packages, setup

package_name = 'modbus_node'

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
    maintainer='h',
    maintainer_email='hasanismaeel53@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
entry_points={
        'console_scripts': [
            'read_speed_values = modbus_node.read_speed_values:main',
            'write_speed_syn_differential_manual = modbus_node.write_speed_syn_differential_manual:main',
            'read_distance = modbus_node.read_distance:main',
            'write_speed_sync_sub_cmdvel = modbus_node.write_speed_sync_sub_cmdvel:main',
            'read_distance_pub_odom = modbus_node.read_distance_pub_odom:main',
            'write_speed_max = modbus_node.write_speed_max:main',
            'reset_distance = modbus_node.reset_distance:main',
            'send_raw_modbus_series = modbus_node.send_raw_modbus_series:main',
        ],
    },
)


