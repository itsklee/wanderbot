from setuptools import setup

package_name = 'wanderbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itsklee',
    maintainer_email='itsklee@gmail.com',
    description='Wanderbot Sample Program for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wanderbot_tele = wanderbot.red_light_green_light:main',
            'wanderbot_scan = wanderbot.range_ahead:main',
            'wanderbot_wander = wanderbot.wander:main',
        ],
    },
)
