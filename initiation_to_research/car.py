from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64, Int32
import numpy as np
import rclpy
from rclpy.node import Node


# ----------------------- ROS2 Car Node -----------------------


class CarNode(Node):

	def __init__(self):
		super().__init__('Car')
		
		self.u, self.dt, self.v = 0., 0.1, 0.  # command (angular velocity)
		self.X = np.array([[0], [0], [0]]) # state vector: x, y, teta
		self.stopped = False

		self.publisher_state = self.create_publisher(Pose2D, 'state', 10)  # queuesize=10
		self.publisher_cost = self.create_publisher(Float64, 'cost', 10)  # queuesize=10
		self.subscriber_yaw = self.create_subscription(Float64, 'command_yaw', self.callback_yaw, 0)
		self.subscriber_velocity = self.create_subscription(Float64, 'velocity', self.callback_velocity, 0)
		self.subscriber_stop = self.create_subscription(Int32, 'stop', self.callback_stop, 0)
		self.subscriber_kill = self.create_subscription(Int32, 'kill', self.callback_kill, 0)
		self.travelled_distance = 0
		self.timer = self.create_timer(self.dt, self.timer_callback)

		self.get_logger().info('Initialisation complete')

	# ----------------------- Callback for ROS Topics -----------------------

	def timer_callback(self):
		self.X = self.X + self.dt * self.f()  # euler

		state = Pose2D() # état du bateau pour le controlleur
		# state.header.stamp = self.get_clock().now().to_msg()
		state.x = self.X[0, 0]
		state.y = self.X[1, 0]
		state.theta = self.X[2, 0]  # yaw

		self.publisher_state.publish(state)

	def callback_yaw(self, msg):
		self.u = msg.data  # angular velocity (yaw) 

	def callback_velocity(self, msg):
		self.v = msg.data  # linear velocity

	def callback_stop(self, msg):
		if not self.stopped: self.publisher_cost.publish(Float64(data=self.travelled_distance))
		self.stopped = True

	def callback_kill(self, msg):
		del self 
		self.destroy_node()
		rclpy.shutdown()

	# ----------------------- State evolution -----------------------

	def f(self):
		teta = self.X[2, 0]  # no velocity regulation
		if self.stopped: self.v, self.u = 0., 0.
		
		self.travelled_distance = self.travelled_distance + self.dt * self.v

		Xdot = np.array([[self.v*np.cos(teta)], [self.v*np.sin(teta)], [self.u]])  # state equation
		return Xdot


# ----------------------- Main program -----------------------


def main(args=None):
	rclpy.init(args=args)
	
	node = CarNode()
	rclpy.spin(node)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
