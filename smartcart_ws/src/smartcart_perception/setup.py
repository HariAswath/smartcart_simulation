from setuptools import find_packages, setup

package_name = 'smartcart_perception'

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
    maintainer='sai',
    maintainer_email='sai@smartcart.local',
    description='Hybrid Multi-Person Perception, BLE, Re-ID, and Target Selection for SmartCart',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ble_simulator = smartcart_perception.ble_simulator:main',
            'target_selector = smartcart_perception.target_selector:main',
        ],
    },
)
