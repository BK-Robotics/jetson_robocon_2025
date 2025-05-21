from setuptools import find_packages, setup

package_name = 'main_controller_pkg'

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
    maintainer='thinh',
    maintainer_email='thinh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_controller_node = main_controller_pkg.main_controller_node:main',
            'full_calculation_node = main_controller_pkg.full_calculation_node:main'
        ],
    },
)
