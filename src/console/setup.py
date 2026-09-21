from setuptools import setup

package_name = 'ap1_console'  # ROS package name

setup(
    name=package_name,                  # Python distribution name
    version='0.1.3',
    # Python package in this repo
    packages=['console'],
    # Include style.css inside the installed console package
    package_data={'console': ['style.css']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'PyQt6'],
    zip_safe=True,
    maintainer='obaidmm',
    maintainer_email='you@example.com',
    description='AP1 console',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run ap1_console console
            'console = console.main:main',
        ],
    },
)
