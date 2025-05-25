from setuptools import setup
from glob import glob

package_name = 'main_controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/models', glob('models/*')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your name',
    maintainer_email='your.email@example.com',
    description='Description of the package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_controller_node = main_controller_pkg.main_controller_node:main',
            'full_calculation_node = main_controller_pkg.full_calculation_node:main'
        ],
    },
)