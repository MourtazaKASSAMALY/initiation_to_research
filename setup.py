from setuptools import setup

package_name = 'initiation_to_research'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", ['launch/launch_mcts_simulations.launch.py']),
        ('share/' + package_name + "/launch", ['launch/launch_aco_simulations.launch.py']),
        ('share/' + package_name + "/launch", ['launch/launch_logger.launch.py']),
        ('share/' + package_name + "/launch", ['launch/launch_mcts_simulations.sh']),
        ('share/' + package_name + "/launch", ['launch/launch_aco_simulations.sh']),
        ('share/' + package_name + "/launch", ['data/scenarios.txt']),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mourtaza',
    maintainer_email='mourtaza.kassamaly@hotmail.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts':[
            'car = initiation_to_research.car:main',
            'controller = initiation_to_research.controller:main',
            'display = initiation_to_research.display:main',
            'mctsplanner = initiation_to_research.mctsplanner:main',
            'acoplanner = initiation_to_research.acoplanner:main',
            'logger = initiation_to_research.logger:main',
        ],
    },
)
