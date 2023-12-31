from setuptools import find_packages, setup

package_name = 'environment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eliot',
    maintainer_email='eliot.coulon@reseau.eseo.fr',
    description='Module de gestion Camera/LIDAR/acoustique',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'environment = src.environment:main'
        ],
    },
)
