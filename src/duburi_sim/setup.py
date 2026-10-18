from glob import glob

from setuptools import setup

package_name = 'duburi_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/arenas', glob('config/arenas/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yaad Kamrul Bari',
    maintainer_email='yaadbari21@gmail.com',
    description='Stand-in Duburi AUV simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = duburi_sim.controller_node:main',
            'vehicle_server = duburi_sim.vehicle_server:main',
            'battery_sim = duburi_sim.battery_sim:main',
            'detector_node = duburi_sim.detector_node:main',
            'referee_node = duburi_sim.referee_node:main',
            'save_camera_frame = duburi_sim.save_camera_frame:main',
            'world_gen = duburi_sim.world_gen:main',
        ],
    },
)
