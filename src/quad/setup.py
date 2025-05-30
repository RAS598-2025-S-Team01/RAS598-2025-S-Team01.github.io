from setuptools import find_packages, setup

package_name = 'quad'

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
    maintainer='jhebbalm',
    maintainer_email='jhebbalm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'connect = quad.nodes.load_esp32:main',
                'plot = quad.nodes.load_plotter:main',
                'app = quad.web.app:main',
                'ur5 = quad.nodes.ur5_node:main',
        ],
    },
)
