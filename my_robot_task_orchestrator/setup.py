from setuptools import setup

package_name = 'my_robot_task_orchestrator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/orchestrator.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cxl',
    maintainer_email='changxianli6121@163.com',
    description='M9 task orchestrator (navigate-detect-pick-place state machine)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'task_orchestrator = my_robot_task_orchestrator.orchestrator:main',
        ],
    },
)
