from setuptools import find_packages, setup

package_name = 'pkg_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/config', ['config/gripper.config.yaml']),
        (f'share/{package_name}/launch', ['launch/manipulation.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='mtrn',
    maintainer_email='alanchoi.uni@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grip_manip = pkg_manipulation.grip_manip:main',
            'test_gripper_client = pkg_manipulation.test_gripper_client:main',
            'manual_gripper_control = pkg_manipulation.manual_gripper_control:main',
        ],
    },
)
