from setuptools import setup

package_name = 'door_adapter_megazo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bey Hao Yun',
    maintainer_email='gary.bey@kabam.ai',
    description='A RMF door adapter for Megazo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'door_adapter = door_adapter_megazo.door_adapter:main'
        ],
    },
)
