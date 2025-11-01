from setuptools import find_packages, setup

package_name = 'pkg_brain'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/brain.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mtrn',
    maintainer_email='alanchoi.uni@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_brain = pkg_brain.task_brain:main',
        ],
    },
)
