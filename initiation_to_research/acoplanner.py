from geometry_msgs.msg import Pose2D, Point
import numpy as np
from initiation_to_research.ACOClass import ACOClass
from std_msgs.msg import Float64MultiArray, Int32
import rclpy
from rclpy.node import Node
import time
import pathlib
import json


# ----------------------- ROS2 Planner Node -----------------------


class PlannerNode(Node):

	def __init__(self):
		super().__init__('Planner')

		time.sleep(1)  # wait other nodes to fully start before planning a mission
		self.subscriber_scenario = self.create_subscription(Int32, '/scenario', self.callback_scenario, 0)
		
		self.get_logger().info('Initialisation complete')

	# ----------------------- Callback for ROS Topics -----------------------

	def callback_scenario(self, msg):

		path = str(pathlib.Path(__file__).parent.absolute().parent.absolute()) + "/data/" + "scenarios.txt"
		self.scenario_index = msg.data
		with open(path, 'r') as f: 
			targets_scenarios = json.loads(f.read())

		for i in range(len(targets_scenarios)):
			for j in range(len(targets_scenarios[i])):
				targets_scenarios[i][j] = np.array(targets_scenarios[i][j])

		self.targets = targets_scenarios[self.scenario_index] + [np.array([[0.], [0.]])]
	
		acs = ACOClass(mode='ACS', colony_size=5, steps=50, nodes=self.targets)
		acs.run()
		self.tour = [acs.nodes[i] for i in acs.global_best_tour]
		
		for i in range(len(self.tour)):  # Find the index of the next waypoint after np.array([[0.], [0.]]) and rotate tour
			if (self.tour[i] == np.array([[0.], [0.]])).all():
				self.tour = self.tour[i+1:] + self.tour[:i+1]
		
		self.wpindex = 0;
		self.phat = self.tour[self.wpindex]  # target location
		
		self.publisher_target = self.create_publisher(Point, 'target', 10)  # queuesize=10
		self.publisher_targets = self.create_publisher(Float64MultiArray, 'targets', 10)  # queuesize=10
		self.publisher_stop = self.create_publisher(Int32, 'stop', 10)  # queuesize=10
		self.subscriber_target_reached = self.create_subscription(Int32, 'target_reached', self.callback_target_reached, 0)
		self.subscriber_kill = self.create_subscription(Int32, 'kill', self.callback_kill, 0)

		self.publisher_target.publish(Point(x = self.phat[0, 0], y = self.phat[1, 0]))
		self.send_targets()

	def callback_target_reached(self, msg):

		# if the origin point has been reached then send a signal to stop the car
		if (self.phat == np.array([[0.], [0.]])).all(): 
			self.publisher_stop.publish(Int32(data=0))

		# remove reached target from targets list
		for i in range(len(self.targets)):
			if (self.targets[i] == self.phat).all(): 
				self.targets.pop(i)
				break

		# send a target
		if self.wpindex < len(self.tour)-1:  
			self.wpindex = (self.wpindex + 1) % len(self.tour)
			self.phat = self.tour[self.wpindex]  # target location
			self.publisher_target.publish(Point(x=self.phat[0, 0], y=self.phat[1, 0]))

		# send all targets list
		self.send_targets()
		
	def callback_kill(self, msg): 
		del self
		self.destroy_node()
		rclpy.shutdown()

	# ----------------------- Functions -----------------------

	def send_targets(self):
		all = [float(len(self.targets))]

		for target in self.targets:
			all.append(target[0, 0])
			all.append(target[1, 0])

		self.publisher_targets.publish(Float64MultiArray(data=tuple(all)))


# ----------------------- Main program -----------------------


def main(args=None):
	rclpy.init(args=args)
	
	node = PlannerNode()
	rclpy.spin(node)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
