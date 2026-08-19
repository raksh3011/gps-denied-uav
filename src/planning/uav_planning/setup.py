from setuptools import find_packages, setup

package_name = 'uav_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Person 3',
    maintainer_email='team@example.com',
    description='Planning module (mock + real global/local planners).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_planner = uav_planning.mock_planner:main',
        ],
    },
)
