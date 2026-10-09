from setuptools import find_packages, setup

package_name = 'drn_perception_reference_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/drn_perception_reference_detection_launch.py']),
        ('share/' + package_name, ['launch/cli_dock_detector_launch.py']),
        ('share/' + package_name, ['launch/experiment.launch.py']),
        ('lib/' + package_name + '/python', ['python/dock_detection.py']),
        ('lib/' + package_name + '/python', ['python/pcd_buffer.py']),
        ('lib/' + package_name + '/python', ['python/dock_detector_client.py']),
        ('share/' + package_name + '/config', ['config/dock_detector_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='allgayer',
    maintainer_email='rafael.allgayer@senairs.org.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            f'dock_detector_service_node = {package_name}.dock_detector_service_node:main',
            f'dock_detector_client_node = {package_name}.dock_detector_client_node:main',
            f'experiment_node = {package_name}.experiment_node:main',
        ],
    },
)
