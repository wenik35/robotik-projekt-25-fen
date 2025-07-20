from setuptools import find_packages, setup

package_name = 'turtlebot_pastry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/latest.py']),
        ('share/' + package_name, ['package.xml', 'launch/TrafficlightLaunch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nik',
    maintainer_email='niwer0305@gmx.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = turtlebot_pastry._state_machine:main',
            'stop = turtlebot_pastry._stop_driving:main',
            'change_lane = turtlebot_pastry.change_lane:main',
            'follow_path = turtlebot_pastry.follow_path:main',
            'trafficlight_start = turtlebot_pastry.trafficlight_start:main',
            'sign_recognition = turtlebot_pastry.sign_recognition:main',
            'change_lane = turtlebot_pastry.change_lane:main',
            'parking = turtlebot_pastry.parking:main',
            'crossing = turtlebot_pastry.crossing:main',
        ],
    },
)
