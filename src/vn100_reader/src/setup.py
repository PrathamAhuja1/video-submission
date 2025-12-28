from setuptools import setup

package_name = 'vn100_reader'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'vectornav',     # VectorNav Python SDK
        'pyserial'
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@email.com',
    description='ROS 2 VN-100 IMU driver publishing YPR and YPR rate',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vn100_ypr_node = vn100_reader.vn100_ypr_publisher:main',
        ],
    },
)

