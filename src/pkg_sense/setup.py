from setuptools import find_packages, setup

package_name = 'pkg_sense'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/config', ['config/sense.config.yaml', 'config/hsv_ranges.default.yaml']),
        (f'share/{package_name}/launch', [
            'launch/sense.launch.py',
            'launch/test_hsv_sense.launch.py',
            'launch/test_hsv_simple.launch.py'
        ]),
        (f'share/{package_name}/test_images', [
            'test_images/klotski_test.jpg'
        ]),
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
            'mock_sense = pkg_sense.scripts.mock_sense_node:main',
            'sense = pkg_sense.sense_node:main',
            'test_hsv_sense = pkg_sense.scripts.test_hsv_sense:main',
            'mock_camera_pub = pkg_sense.scripts.mock_camera_pub:main'
        ],
    },
)
