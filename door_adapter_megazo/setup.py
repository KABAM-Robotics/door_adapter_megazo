from setuptools import setup, find_packages

package_name = 'door_adapter_megazo'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/run.py']),
    ],
    install_requires=[
        'setuptools',
        'rclpy'
        ],
    zip_safe=True,
    maintainer='Bey Hao Yun',
    maintainer_email='gary.bey@kabam.ai',
    description='A RMF door adapter for Megazo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'door_adapter = door_adapter_megazo.door_adapter:main',
            'mqtt_client = door_adapter_megazo.mqtt_client:main'
        ],
    },
)
