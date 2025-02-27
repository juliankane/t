from setuptools import find_packages, setup

package_name = 'services'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/service_materials', [
            'service_materials/hospital.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='julin',
    maintainer_email='juliankane@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_to_pose_service = services.map_to_pose_service:main',
            'map_to_pose_client = services.map_to_pose_client:main',
        ],
    },
)
