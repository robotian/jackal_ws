from setuptools import find_packages, setup

package_name = 'jackel_ops'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/launch', glob('launch/*.py')),
        # ('share/' + package_name + '/config', glob('config/*')),
        # ('share/' + package_name + '/data', glob('data/*')),
    ],
    install_requires=['setuptools','launch'],
    zip_safe=True,
    maintainer='Vinay Upadhye',
    maintainer_email='vupadhye@mtu.edu',
    description='ROS 2 Python package for monitoring robot status, executing tasks, and reporting progress and results.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_status = jackel_ops.status:main',
        ],
    },
)
