from setuptools import find_packages, setup

package_name = 'uav_localization'

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
    maintainer='Person 1',
    maintainer_email='team@example.com',
    description='Localization module (mock + real LIO).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_localization = uav_localization.mock_localization:main',
        ],
    },
)
