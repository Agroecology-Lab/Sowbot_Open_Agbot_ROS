from setuptools import find_packages, setup

package_name = 'basekit_driver'

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
    maintainer='Agroecology Lab',
    maintainer_email='lab@agroecology.org',
    description='Driver for Open Agbot basekit hardware',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basekit_driver_node = basekit_driver.basekit_driver_node:main'
        ],
    },
)
