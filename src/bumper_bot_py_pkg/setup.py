from setuptools import find_packages, setup

package_name = 'bumper_bot_py_pkg'

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
    maintainer='sunil',
    maintainer_email='sunilkumarb1811@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_publisher = bumper_bot_py_pkg.simple_publisher_file:main',
            'simple_subscriber = bumper_bot_py_pkg.simple_subscriber_file:main',  
            'simple_parameter = bumper_bot_py_pkg.simple_parameter:main',
            'simple_turtlesim_kinematics = bumper_bot_py_pkg.turtlesim_kinematics:main'          
        ],
    },
)
