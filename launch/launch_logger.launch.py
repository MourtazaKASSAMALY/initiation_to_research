import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

	logger_node = Node(
		package='initiation_to_research',
		output='screen',
		node_executable='logger',
		emulate_tty=True
	)

	return LaunchDescription([logger_node])

