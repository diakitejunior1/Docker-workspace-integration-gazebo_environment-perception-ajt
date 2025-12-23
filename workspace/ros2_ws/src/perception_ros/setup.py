from setuptools import setup

package_name = 'perception_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        'Perception',
        'Perception.model',
        'Perception.utils'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emav',
    maintainer_email='emav@todo.todo',
    description='Perception-based control stack',
    license='TODO',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            # Existing nodes
            'perception = perception_ros.perception:main',
            'controller = perception_ros.controller:main',

            # ✅ Perception-based controllers (MATCH FILE NAMES)
            'perception_pid_controller = perception_ros.perception_pid_controller:main',
            'perception_pure_pursuit = perception_ros.perception_pure_pursuit:main',
        ],
    },
)
