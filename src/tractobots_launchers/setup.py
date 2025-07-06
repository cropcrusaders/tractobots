from glob import glob
from setuptools import setup, find_packages

package_name = 'tractobots_launchers'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicholas Bass',
    maintainer_email='nicholasbass@crop-crusaders.com',
    description='Top-level launch files that start the whole Tractobots stack.',
    license='GPLv3',
    entry_points={},
)
