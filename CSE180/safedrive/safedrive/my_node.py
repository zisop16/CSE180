import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from time import time



class SafeNode(Node):
	id = '100445185'
	N = int(id[-1])
	zero_vec = Vector3()
	zero_vec.x, zero_vec.y, zero_vec.z = (0., 0., 0.)
	zero_twist = Twist()
	zero_twist.linear = zero_vec
	zero_twist.angular = zero_vec


	def __init__(self):
		super().__init__("SafeNode")
		self.most_recent_cmd_vel = 0
		self.get_logger().info("Hello universe")
		self.cmd_vel_sub = self.create_subscription(Twist, "/turtle1/cmd_vel", self.cmd_vel_callback, 1)
		self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)
		self.currently_moving = False
		self.most_recent_twist = SafeNode.zero_twist

		publish_hertz = 10
		publish_delay = 1./publish_hertz
		self.timer = self.create_timer(publish_delay, self.timer_callback)

	def cmd_vel_callback(self, msg: Twist):
		
		v_max = .1 + .02 * SafeNode.N
		w_max = .4 + .05 * SafeNode.N
		

		linear = msg.linear
		angular = msg.angular

		new_lin_x = clamp(linear.x, -v_max, v_max)
		new_ang_z = clamp(angular.z, -w_max, w_max)

		linear.x = new_lin_x
		angular.z = new_ang_z

		self.most_recent_cmd_vel = time()
		self.most_recent_twist = msg

	def timer_callback(self):
		timeout = .5 + .05 * SafeNode.N
		curr_time = time()
		time_delta = curr_time - self.most_recent_cmd_vel

		to_publish = self.most_recent_twist

		prev_moving = self.currently_moving
		if time_delta > timeout:
			to_publish = SafeNode.zero_twist
			self.currently_moving = False
		else: 
			self.currently_moving = True

		moving_changed = False
		if prev_moving != self.currently_moving:
			moving_changed = True
		if moving_changed:
			if not self.currently_moving:
				self.get_logger().info("Robot has timed out, movement stopped")
			else:
				self.get_logger().info("Robot has resumed moving")

		self.cmd_vel_pub.publish(to_publish)
		


def clamp(val: float, lower: float, upper: float):
	if val < lower:
		val = lower
	if val > upper:
		val = upper
	return val


def main(args=None):
	rclpy.init(args=args)
	node = SafeNode()
	rclpy.spin(node)


	rclpy.shutdown()

if __name__ == '__main__':
	main()