from setuptools import find_packages, setup

package_name = 'smartcart_human'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hariaswath',
    maintainer_email='hariaswath@todo.todo',
    description='Human simulation, follow controller, and RFID simulator for SmartCart',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_controller = smartcart_human.human_controller:main',
            'follow_controller = smartcart_human.follow_controller:main',
            'rfid_simulator = smartcart_human.rfid_simulator:main',
        ],
    },
)
