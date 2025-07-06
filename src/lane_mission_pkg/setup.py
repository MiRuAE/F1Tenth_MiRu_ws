from setuptools import find_packages, setup

package_name = 'lane_mission_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'ultralytics>=8.3',
        'torch'
    ],
    zip_safe=True,
    maintainer='shchon11',
    maintainer_email='shchon11@naver.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_mission_node = lane_mission_pkg.lane_mission_node:main'
        ],
    },
)
