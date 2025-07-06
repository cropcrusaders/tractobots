from setuptools import setup
package_name = 'tractobots_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicholas Bass',
    maintainer_email='nicholasbass@crop-crusaders.com',
    description='Tele‑op and autonomous driver for Tractobots',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'driver = tractobots_navigation.driver:main',
        ],
    },
)
