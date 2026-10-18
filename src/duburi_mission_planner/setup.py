from glob import glob

from setuptools import setup

package_name = 'duburi_mission_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.tools', package_name + '.llm', package_name + '.backends'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yaad Kamrul Bari',
    maintainer_email='yaadbari21@gmail.com',
    description='LLM mission planner for the Duburi AUV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission = duburi_mission_planner.cli:main',
            'mission_server = duburi_mission_planner.mission_server_node:main',
            'tool_demo = duburi_mission_planner.tool_demo:main',
            'duburi_adapter = duburi_mission_planner.backends.duburi_adapter:main',
        ],
    },
)
