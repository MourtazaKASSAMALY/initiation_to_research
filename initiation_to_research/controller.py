import numpy as np
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Float64, Float64MultiArray, Int32
from numpy.linalg import norm
import rclpy
from rclpy.node import Node


# ----------------------- ROS2 Controller Node -----------------------


class ControllerNode(Node):

	def __init__(self):
		super().__init__('Controller')

		self.posx, self.posy, self.yaw = 0., 0., 0.  # state of the car
		self.phat = np.array([[0.], [0.]])  # target location

		self.publisher_yaw = self.create_publisher(Float64, 'command_yaw', 10)  # queuesize=10
		self.publisher_target_reached = self.create_publisher(Int32, 'target_reached', 10)  # queuesize=10 
		self.publisher_velocity = self.create_publisher(Float64, 'velocity', 10)  # queuesize=10
		self.subscriber_state = self.create_subscription(Pose2D, 'state', self.callback_state, 0)
		self.subscriber_target = self.create_subscription(Point, 'target', self.callback_target, 0)
		self.subscriber_kill = self.create_subscription(Int32, 'kill', self.callback_kill, 0)

		self.get_logger().info('Initialisation complete')

	# ----------------------- Functions -----------------------

	def sawtooth(self, x): return (x + np.pi) % (2 * np.pi) - np.pi   # or equivalently   2*arctan(tan(x/2))

	# ----------------------- Callback for ROS Topics -----------------------

	def callback_state(self, msg):  # car state listener
		self.posx, self.posy, self.yaw = msg.x, msg.y, msg.theta

		p = np.array([[self.posx], [self.posy]])		
		
		w = - 2 * (p - self.phat)
		
		tetabar = np.arctan2(w[1, 0], w[0, 0])
		u = 5 * self.sawtooth(tetabar - self.yaw)
		
		dist_to_waypoint = np.linalg.norm(p-self.phat)
		if dist_to_waypoint <= 0.2: 
			self.publisher_target_reached.publish(Int32(data=0))

		self.publisher_yaw.publish(Float64(data=u))  # angular velocity command (yaw)

	def callback_target(self, msg):  # target location
		self.phat[0, 0] = msg.x
		self.phat[1, 0] = msg.y
		self.publisher_velocity.publish(Float64(data=1.))

	def callback_kill(self, msg):
		del self
		self.destroy_node()
		rclpy.shutdown()


# ----------------------- Main program -----------------------


def main(args=None):
	rclpy.init(args=args)

	node = ControllerNode()
	rclpy.spin(node)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
