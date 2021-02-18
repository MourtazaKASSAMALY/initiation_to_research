import numpy as np
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Float64MultiArray, Int32
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

np.seterr(divide='ignore', invalid='ignore')  
# avoid warning when a value of the meshgrid X1, X2 is equal to an obstacle position (when dividing by norm draw_field function)


# ----------------------- ROS2 Display Node -----------------------


class DisplayNode(Node):

	def __init__(self):
		super().__init__('Display')

		self.display = True

		self.posx, self.posy, self.v, self.yaw = 0, 0, 0, 0  # state of the car
		self.phat = np.array([[0.], [0.]])  # target location
		self.targets = []

		# Initialize and clear ax
		"""self.xmin, self.xmax, self.ymin, self.ymax = -1, 16, -1, 16
		self.fig = plt.figure(0)
		self.ax = self.fig.add_subplot(111, aspect='equal')	
		self.ax.xmin, self.ax.xmax, self.ax.ymin, self.ax.ymax = self.xmin, self.xmax, self.ymin, self.ymax
		plt.pause(0.001)
		plt.cla()
		self.ax.set_xlim(self.ax.xmin, self.ax.xmax)
		self.ax.set_ylim(self.ax.ymin, self.ax.ymax)"""

		self.subscriber_state = self.create_subscription(Pose2D, 'state', self.callback_state, 0)
		self.subscriber_target = self.create_subscription(Point, 'target', self.callback_target, 0)
		self.subscriber_targets = self.create_subscription(Float64MultiArray, 'targets', self.callback_targets, 0)
		self.subscriber_kill = self.create_subscription(Int32, 'kill', self.callback_kill, 0)

		self.get_logger().info('Initialisation complete')

	# ----------------------- Drawing functions -----------------------
	"""
	# Dessin du champ de vecteurs
	def draw_field(self):
		Mx = np.arange(self.xmin, self.xmax, 0.5)
		My = np.arange(self.ymin, self.ymax, 0.5)
		X1, X2 = np.meshgrid(Mx, My)

		VX = - 2 * (X1 - self.phat[0])
		VY = - 2 * (X2 - self.phat[1])

		R = np.sqrt(VX**2 + VY**2)  # normalisation
		
		self.ax.quiver(Mx, My, VX/R, VY/R)

	def move_motif(self, M, x, y, teta):
		M1 = np.ones((1, len(M[1,:])))
		M2 = np.vstack((M, M1))
		R = np.array([[np.cos(teta), -np.sin(teta), x], [np.sin(teta), np.cos(teta), y]])
		return(R.dot(M2))    

	def draw_tank(self, col='darkblue', r=1, w=2):
		x = np.array([[self.posx], [self.posy], [self.yaw]])  # voiture : x, y, θ

		plt.pause(0.001)
		plt.cla()
		self.ax.set_xlim(self.ax.xmin, self.ax.xmax)
		self.ax.set_ylim(self.ax.ymin, self.ax.ymax)

		x = x.flatten()
		M = r * np.array([[1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0], [-2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1]])
		M = self.move_motif(M, x[0], x[1], x[2])

		self.ax.plot(M[0, :], M[1, :], col, linewidth=w) 

	def draw_objectives(self):
		for target in self.targets: self.ax.plot(target[0, 0], target[1, 0], "go", markersize=10)  # targets
		self.ax.plot(self.phat[0, 0], self.phat[1, 0], 'go', markeredgecolor='black', markersize=10)  # target
		self.ax.plot(0, 0, 'bo', markersize=10)  # origin
	"""
	# ----------------------- Callback for ROS Topics -----------------------

	def callback_state(self, msg):  # car state listener
		self.posx = msg.x
		self.posy = msg.y
		self.yaw = msg.theta  # yaw

		# self.draw_tank('red', 0.2)
		# self.draw_field()  # champ de vecteurs
		# self.draw_objectives()  # dessin des obstacles et des cibles 

	def callback_target(self, msg):  # target location 
		self.phat[0, 0] = msg.x
		self.phat[1, 0] = msg.y

	def callback_targets(self, msg):
		ntargets = int(msg.data[0])
		parsed_targets = []
		
		for i in range(ntargets):
			reading = np.array([ [msg.data[1 + 2*i]], [msg.data[1 + 2*i + 1]] ]) 
			parsed_targets.append(reading)
		
		self.targets = parsed_targets
		
	def callback_kill(self, msg):
		plt.close()
		del self
		self.destroy_node()
		rclpy.shutdown()


# ----------------------- Main program -----------------------


def main(args=None):
	rclpy.init(args=args)

	node = DisplayNode()
	rclpy.spin(node)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
