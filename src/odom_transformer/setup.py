from setuptools import find_packages, setup
from glob import glob

package_name = 'odom_transformer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotian',
    maintainer_email='robotian@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_transformer_node = odom_transformer.odom_transformer_node:main',
            'odom_transformer_ekf_node = odom_transformer.odom_transformer_ekf_node:main', 
            # 'odom_heading_publisher = odom_transformer.odom_heading_publisher:main', #odom_heading_publisher.py
            'dual_gps_odom_publisher = odom_transformer.dual_gps_odom_publisher:main', 
            'attitude_duro_heading2odom = odom_transformer.attitude_duro_heading2odom:main',
        ],
    },
)
