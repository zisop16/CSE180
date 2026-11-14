import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from time import time

import numpy as np
from PIL import Image
from sensor_msgs.msg import Image as ROSImage

zero_vec = Vector3()
zero_vec.x, zero_vec.y, zero_vec.z = (0., 0., 0.)
zero_twist = Twist()
zero_twist.linear = zero_vec
zero_twist.angular = zero_vec



def depth_to_numpy(msg: ROSImage) -> np.ndarray:
    """32FC1 ROS2 Image -> (height, width) float32 array."""
    dtype = np.dtype(np.float32).newbyteorder(">" if msg.is_bigendian else "<")
    row_floats = msg.step // 4  # step may include row padding
    arr = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_floats)
    return arr[:, :msg.width].astype(np.float32)

def save_depth(msg: ROSImage, vis_path="detector/depth.png"):
    depth = depth_to_numpy(msg)

    # Lossless: real float values (TIFF supports 32-bit float, PNG does not)
    # Image.fromarray(depth, mode="F").save(raw_path)

    # 8-bit grayscale for viewing (NaN/inf/<=0 -> black)
    valid = np.isfinite(depth) & (depth > 0)
    vis = np.zeros(depth.shape, dtype=np.uint8)
    if valid.any():
        lo, hi = depth[valid].min(), depth[valid].max()
        vis[valid] = ((depth[valid] - lo) / max(hi - lo, 1e-9) * 255).astype(np.uint8)
    Image.fromarray(vis, mode="L").save(vis_path)

def ros_image_to_pil(msg: ROSImage) -> Image.Image:
    # Respect row padding (step) before cropping to width
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    arr = rows[:, : msg.width * 3].reshape(msg.height, msg.width, 3)

    return Image.fromarray(arr, mode="RGB")


def save_rgb(msg: ROSImage, path="detector/rgb.png") -> None:
    ros_image_to_pil(msg).save(path, format="PNG")


class DetectorNode(Node):
	def __init__(self):
		super().__init__("detector_node")
		self.get_logger().info("Detector Online.")
	
		self.depth_image_sub = self.create_subscription(ROSImage, "/rgbd_camera/depth_image", self.depth_image_callback, 1)
		self.rgb_image_sub = self.create_subscription(ROSImage, "/rgbd_camera/image", self.rgb_image_callback, 1)

	def depth_image_callback(self, img: ROSImage):
		print(f"({img.width}, {img.height})")
		print(len(img.data), img.encoding, img.is_bigendian)
		save_depth(img)

	def rgb_image_callback(self, img: ROSImage):
		print(f"({img.width}, {img.height})")
		print(len(img.data), img.encoding, img.is_bigendian)
		save_rgb(img)

	"""
		self.currently_moving = False
		self.most_recent_twist = SafeNode.zero_twist

		publish_hertz = 10
		publish_delay = 1./publish_hertz
		self.timer = self.create_timer(publish_delay, self.timer_callback)

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
	"""

def clamp(val: float, lower: float, upper: float):
	if val < lower:
		val = lower
	if val > upper:
		val = upper
	return val


def main(args=None):
	rclpy.init(args=args)
	node = DetectorNode()
	rclpy.spin(node)


	rclpy.shutdown()

if __name__ == '__main__':
	main()