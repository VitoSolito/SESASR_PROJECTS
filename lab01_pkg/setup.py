from setuptools import find_packages, setup

package_name = 'lab01_pkg'

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
    maintainer='vito',
    maintainer_email='s344100@studenti.polito.it',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = lab01_pkg.controller:main',
            'listener = lab01_pkg.localization:main',
            'reset= lab01_pkg.reset_node:main',
            'reset_controller= lab01_pkg.reset_controller_fcn:main',
            'reset_localization= lab01_pkg.reset_localization_fcn:main',
        ],
    },
)
