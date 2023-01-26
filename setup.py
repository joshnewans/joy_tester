from setuptools import setup

package_name = 'joy_tester'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Josh Newans',
    maintainer_email='josh.newans@gmail.com',
    description='Simple GUI tool for testing joysticks/gamepads',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_joy = joy_tester.test_joy:main',
        ],
    },
)
