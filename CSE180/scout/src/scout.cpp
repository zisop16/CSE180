#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>

using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace rclcpp;
using namespace std;
using namespace sensor_msgs::msg;
using namespace tf2_ros;
using namespace std::chrono_literals;


Twist ZERO_TWIST;
Point ZERO_POINT;
Vector3 ZERO_VEC;

Node::SharedPtr nodeh;
Buffer* transform_buffer;
shared_ptr<Publisher<Twist, std::allocator<void>>> twistPublisher;

// Robot is base_link
// odometry is odom
// scanner is rplidar_link
// Transform base_link -> rplidar_link is 
// [0 -1 0 -.04]
// [1 0 0 0]
// [0 0 1 0.193]
// [0 0 0 1]


bool scanToProcess = false;
vector<float> recentRanges;
float angle_min;
float angle_max;
float angle_inc;
float range_min;
float range_max;
double N = 5;

void scanReceived(const LaserScan::SharedPtr msg) {
	vector<float> ranges = msg->ranges;
	angle_min = msg->angle_min;
	angle_max = msg->angle_max;
	angle_inc = msg->angle_increment;
	range_min = msg->range_min;
	range_max = msg->range_max;
	recentRanges = ranges;
	scanToProcess = true;
}

enum scan_phase {
	DETECT, APPROACH, RETURN, DONE
};
scan_phase currPhase = DETECT;

double calculateLength2D(geometry_msgs::msg::PointStamped p) {
	float x, y, z;
	x = p.point.x;
	y = p.point.y;
	return sqrt(x*x + y*y);
}

double calculateAngle(geometry_msgs::msg::PointStamped p) {
	float x, y;
	float len;
	x = p.point.x;
	y = p.point.y;
	len = sqrt(x*x + y*y);
	float theta = acos(x / len);
	if (y < 0) {
		theta = -theta;
	}
	return theta;
}

PointStamped target;
PointStamped home;
void logPoint(PointStamped msg) {
	Point p = msg.point;
	RCLCPP_INFO(nodeh->get_logger(), "(%f, %f, %f)\n", p.x, p.y, p.z);
}
bool recentLookupFailure = false;

bool findRobotClosest(TransformStamped robotTransform, TransformStamped odomTransform) {
	double beta = 20 + 2 * N;
	beta *= M_PI / 180;
	bool foundValid = false;
	PointStamped closestPoint;
	double closestDist;
	for (int i = 0; i < recentRanges.size(); i++) {
		float currRange = recentRanges[i];
		if (currRange > range_max || currRange < range_min) {
			continue;
		}
		geometry_msgs::msg::PointStamped point_in, point_out;
		point_in.point.z = 0;
		float currTheta = i * angle_inc + angle_min;
		point_in.point.x = cos(currTheta) * currRange;
		point_in.point.y = sin(currTheta) * currRange;
		tf2::doTransform(point_in, point_out, robotTransform);
		if (point_out.point.x <= 0) {
			continue;
		}
		currTheta = calculateAngle(point_out);
		if (abs(currTheta) > beta) {
			continue;
		}
		
		double currDist = calculateLength2D(point_out);
		if (!foundValid) {
			closestDist = currDist;
			closestPoint = point_out;
			foundValid = true;
		} else if (currDist < closestDist) {
			closestDist = currDist;
			closestPoint = point_out;
		}
	}
	if (!foundValid) {
		return false;
	}
	tf2::Transform scannerTransform; 
	tf2::fromMsg(robotTransform.transform, scannerTransform);
	scannerTransform = scannerTransform.inverse();
	TransformStamped scannerTransMsg;
	scannerTransMsg.transform = tf2::toMsg(scannerTransform);
	PointStamped originalPoint;
	tf2::doTransform(closestPoint, originalPoint, scannerTransMsg);

	PointStamped odomPoint;
	tf2::doTransform(closestPoint, odomPoint, odomTransform);

	RCLCPP_INFO(nodeh->get_logger(), "Target in each reference frame\n");
	RCLCPP_INFO(nodeh->get_logger(), "Scanner: ");
	logPoint(originalPoint);
	RCLCPP_INFO(nodeh->get_logger(), "Robot: ");
	logPoint(closestPoint);
	RCLCPP_INFO(nodeh->get_logger(), "Odom: ");
	logPoint(odomPoint);

	target = odomPoint;

	return true;
}

void approachTarget() {
	TransformStamped odomToRobot;
	try {
		odomToRobot = transform_buffer->lookupTransform("base_link", "odom", Time(0));
		recentLookupFailure = false;
	}
	catch (tf2::LookupException e) {
		twistPublisher->publish(ZERO_TWIST);
		if (!recentLookupFailure) {
			RCLCPP_INFO(nodeh->get_logger(), "Transform Lookup Failed\n");
			recentLookupFailure = true;
		}
		
		return;
	}
	PointStamped robotPoint;
	tf2::doTransform(target, robotPoint, odomToRobot);
	double dist = calculateLength2D(robotPoint);
	Point goalPosRobot = robotPoint.point;
	Twist currTwist = ZERO_TWIST;
	double d_park = 0.30 + 0.02 * N;
	if (dist >= d_park) {
		double theta;
		if (goalPosRobot.x >= dist) {
			// Random fucking floating point error
			theta = 0;
		} else {
			theta = acos(goalPosRobot.x / dist);
		}
		if (goalPosRobot.y < 0) {
			theta = -theta;
		}
		if (theta > 0) {
			currTwist.angular.z = min(1.0 * theta, .8);
		} else {
			currTwist.angular.z = max(1.0 * theta, -.8);
		}
		
		if (abs(theta) < .3) {
			currTwist.linear.x = min(.5 * dist, .2);
		} else {
			currTwist.linear.x = 0;
		}
	} else {
		currTwist.linear.x = 0;
		currTwist.angular.z = 0;
		RCLCPP_INFO(nodeh->get_logger(), "Arrived at target\n");
		currPhase = RETURN;
	}
	twistPublisher->publish(currTwist);
}

void approachHome() {
	TransformStamped odomToRobot;
	TransformStamped robotToOdom;
	try {
		odomToRobot = transform_buffer->lookupTransform("base_link", "odom", Time(0));
		robotToOdom = transform_buffer->lookupTransform("odom", "base_link", Time(0));
		recentLookupFailure = false;
	}
	catch (tf2::LookupException e) {
		twistPublisher->publish(ZERO_TWIST);
		if (!recentLookupFailure) {
			RCLCPP_INFO(nodeh->get_logger(), "Transform Lookup Failed\n");
			recentLookupFailure = true;
		}
			
		return;
	}
	PointStamped robotPoint;
	tf2::doTransform(home, robotPoint, odomToRobot);
	double dist = calculateLength2D(robotPoint);
	Point goalPosRobot = robotPoint.point;
	Twist currTwist = ZERO_TWIST;
	if (dist >= .10) {
		double theta;
		if (goalPosRobot.x >= dist) {
			// Random fucking floating point error
			theta = 0;
		} else {
			theta = acos(goalPosRobot.x / dist);
		}
		if (goalPosRobot.y < 0) {
			theta = -theta;
		}
		if (theta > 0) {
			currTwist.angular.z = min(1.0 * theta, .8);
		} else {
			currTwist.angular.z = max(1.0 * theta, -.8);
		}
		
		if (abs(theta) < .3) {
			currTwist.linear.x = min(.5 * dist, .2);
		} else {
			currTwist.linear.x = 0;
		}
	} else {
		currTwist.linear.x = 0;
		currTwist.angular.z = 0;
		RCLCPP_INFO(nodeh->get_logger(), "Arrived at home\n");
		logPoint(home);
		currPhase = DONE;
	}
	twistPublisher->publish(currTwist);
}

void timerCallback() {
	switch(currPhase) {
		case (DETECT): {
			if (!scanToProcess) {
				break;
			}
			scanToProcess = false;
			TransformStamped lidar_trans;
			TransformStamped odom_trans;
			try {
				lidar_trans = transform_buffer->lookupTransform("base_link", "rplidar_link", Time(0));
				odom_trans = transform_buffer->lookupTransform("odom", "base_link", Time(0));
				recentLookupFailure = false;
			}
			catch (tf2::LookupException e) {
				if (!recentLookupFailure) {
					RCLCPP_INFO(nodeh->get_logger(), "Transform Lookup Failed\n");
					recentLookupFailure = true;
				}
				break;
			}
			bool foundTarget = findRobotClosest(lidar_trans, odom_trans);
			if (foundTarget) {
				currPhase = APPROACH;
				PointStamped localHome;
				localHome.point.x = 0;
				localHome.point.y = 0;
				localHome.point.z = 0;
				tf2::doTransform(localHome, home, odom_trans);
			}
			else {
				RCLCPP_INFO(nodeh->get_logger(), "No target found.\n");
			}
			break;
		}
		case (APPROACH): {
			
			approachTarget();
			break;
		}
		case (RETURN): {
			approachHome();
			break;
		} 
		case (DONE): {
			twistPublisher->publish(ZERO_TWIST);
			break;
		}
	}
}

int main(int argc, char **argv) {
	init(argc, argv);

	ZERO_VEC.x = ZERO_VEC.y = ZERO_VEC.z = ZERO_POINT.x = ZERO_POINT.y = ZERO_POINT.z = 0;
	ZERO_TWIST.angular = ZERO_VEC;
	ZERO_TWIST.linear = ZERO_VEC;

	rclcpp::Clock::SharedPtr the_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
	Buffer buffer(the_clock);
	TransformListener listener(buffer);
	transform_buffer = &buffer;

	nodeh = Node::make_shared("the_scoutist");
	twistPublisher = nodeh->create_publisher<Twist>("/cmd_vel", 1);
	auto scanSub = nodeh->create_subscription<LaserScan>("/scan", 10, &scanReceived);
	auto wallTimer = nodeh->create_wall_timer(.1s, &timerCallback);
	RCLCPP_INFO(nodeh->get_logger(), "Scoutist Online.\n");
	
	spin(nodeh);
	shutdown();
	return 0;
}