from setuptools import setup

package_name = 'zadanieNMvR'

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
    maintainer='nmvr',
    maintainer_email='nmvr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'map = zadanieNMvR.publisher_map:main',
        	'subscriber = zadanieNMvR.subscriber:main',
        	'smer = zadanieNMvR.publisher_goal:main',
        	'stena = zadanieNMvR.publisher_robot:main'
        ],
    },
)
