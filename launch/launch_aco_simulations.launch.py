import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

	car_node = Node(
		package='initiation_to_research',
		output='screen',
		node_executable='car',
		emulate_tty=True
	)

	controller_node = Node(
		package='initiation_to_research',
		output='screen',
		node_executable='controller',
		emulate_tty=True
	)

	display_node = Node(
		package='initiation_to_research',
		output='screen',
		node_executable='display',
		emulate_tty=True
	)

	acoplanner_node = Node(
		package='initiation_to_research',
		output='screen',
		node_executable='acoplanner',
		emulate_tty=True
	)

	return LaunchDescription([display_node, controller_node, car_node, acoplanner_node])

