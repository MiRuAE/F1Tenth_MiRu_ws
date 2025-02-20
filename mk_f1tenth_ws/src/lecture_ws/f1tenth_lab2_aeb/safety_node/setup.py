from setuptools import setup

package_name = 'f1tenth_aeb'

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
    maintainer='Hongrui Zheng, zzangupenn',
    maintainer_email='billyzheng.bz@gmail.com, zzang@seas.upenn.edu',
    description='Skeleton code for Lab 1: Automatic Emergency Braking at University of Pennsylvania',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'f1tenth_aeb = f1tenth_aeb.f1tenth_aeb:main',
        ],
    },
)
