from setuptools import find_packages, setup

package_name = 'gpg_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['gpg_test/my_robot_controller.py']),
        ('share/'+ package_name, ['gpg_test/my_robot_goal.py']),
        ('share/'+package_name, ['gpg_test/my_line_follower.py']),
        ('share/' + package_name, ['launch/line_launch.py']),
        ('share/' + package_name, ['launch/controller_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lci',
    maintainer_email='lci@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'my_robot_controller=gpg_test.my_robot_controller:main',
        'my_robot_goal=gpg_test.my_robot_goal:main',
        'my_line_follower=gpg_test.my_line_follower:main'
        ],
    },
)


