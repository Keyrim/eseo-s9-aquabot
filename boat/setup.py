from setuptools import find_packages, setup

package_name = 'boat'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/launch_all.launch.py',
            'launch/estimator_controller.launch.py'
        ]),
        # Add more launch files here if you have them
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tmagne',
    maintainer_email='theo.magne.fr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'estimator = boat.estimator:main',
            'controller = boat.controller:main'
        ],
    },
)