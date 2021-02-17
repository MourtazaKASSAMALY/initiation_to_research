import subprocess
from std_msgs.msg import Float64, Int32
import numpy as np
import rclpy
from rclpy.node import Node
import pathlib
import os
import time
from ament_index_python.packages import get_package_share_directory
import json


# ----------------------- ROS2 Car Node -----------------------


class LoggerNode(Node):

	def __init__(self):
		super().__init__('Logger')
		self.publisher_kill = self.create_publisher(Int32, 'kill', 10)  # queuesize=10
		self.subscriber_cost = self.create_subscription(Float64, 'cost', self.callback_cost, 0)
		self.publisher_scenario = self.create_publisher(Int32, 'scenario', 10)

		path = str(pathlib.Path(__file__).parent.absolute().parent.absolute()) + "/data/" + "scenarios.txt"
		with open(path, 'r') as f: targets_scenarios = json.loads(f.read())
		self.max_scenarios = len(targets_scenarios)

		self.scenario_index = 0
		self.costs = []
		self.mode = "ACO"
		# self.mode = "MCTS"

		self.path_mcts = str(pathlib.Path(__file__).parent.absolute().parent.absolute().parent.absolute().parent.absolute()) + \
			"/src/initiation_to_research/data/mcts_results.txt"
		self.path_aco = str(pathlib.Path(__file__).parent.absolute().parent.absolute().parent.absolute().parent.absolute()) + \
			"/src/initiation_to_research/data/aco_results.txt"

		# package_share_directory = get_package_share_directory('initiation_to_research')  # may raise PackageNotFoundError

		self.get_logger().info('Initialisation complete')

		self.launch()

	# ----------------------- Callback for ROS Topics -----------------------

	def callback_cost(self, msg):
		self.publisher_kill.publish(Int32(data=0))

		self.costs.append(msg.data)
		self.scenario_index += 1
		
		if self.scenario_index == self.max_scenarios:
			if self.mode == "ACO": 
				with open(self.path_aco, 'w') as f: f.write(json.dumps(self.costs))
			else: 
				with open(self.path_mcts, 'w') as f: f.write(json.dumps(self.costs))
			
			time.sleep(3)
			print("Done ... type Ctrl-C to exit")
			self.destroy_node()
			rclpy.shutdown()
		else:
			self.launch()

	# ----------------------- Functions -----------------------

	def launch(self):
		if self.mode == "ACO": path = str(pathlib.Path(__file__).parent.absolute().parent.absolute()) + "/launch/" + "launch_aco_simulations.sh"
		else: path = str(pathlib.Path(__file__).parent.absolute().parent.absolute()) + "/launch/" + "launch_mcts_simulations.sh"

		# subprocess.call(['sh', path])  # blocking version
		
		os.system("chmod +x " + path)
		shellscript = subprocess.Popen([path], shell=True)  # non blocking version but requires a chmod +x command first
		time.sleep(4)
		print(self.mode + " Scenario " + str(self.scenario_index+1) + "/" + str(self.max_scenarios))
		self.publisher_scenario.publish(Int32(data=self.scenario_index))


# ----------------------- Main program -----------------------


def main(args=None):
	rclpy.init(args=args)

	node = LoggerNode()
	rclpy.spin(node)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
