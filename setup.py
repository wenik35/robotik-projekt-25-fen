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
            'stateMachine = turtlebot_pastry._stateMachine:main',
            'stop = turtlebot_pastry._stop:main',
            'changeLaneAtObstacle = turtlebot_pastry.changeLaneAtObstacle:main',
            'followPath = turtlebot_pastry.followPath:main',
            'trafficlight_start = turtlebot_pastry.trafficlight_start:main',
            'imageProcessing = turtlebot_pastry._imageProcessing:main',
            'changeLane = turtlebot_pastry.changeLane:main',

        ],
    },
)
