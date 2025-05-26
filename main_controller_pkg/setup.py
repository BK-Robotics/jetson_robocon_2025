from setuptools import setup, find_packages
from glob import glob

package_name = 'main_controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['main_controller_pkg', 'main_controller_pkg.*']),
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
            'full_calculation_node = main_controller_pkg.full_calculation_node:main',
            'capture_image_node = main_controller_pkg.capture_image_node:main',
            'plane_calculation_node = main_controller_pkg.plane_calculation_node:main',
            'tf_publish_node = main_controller_pkg.tf_publish_node:main'
        ],
    },
)