#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
// #include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "quaternions.hpp"

using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace rclcpp;
using namespace std;

Node::SharedPtr nodeh;
shared_ptr<Publisher<Twist, std::allocator<void>>> publisher;

string pointToString(Point p) {
	string coords = "(";
	coords += to_string(p.x) + ", ";
	coords += to_string(p.y) + ", ";
	coords += to_string(p.z) + ")";
	return coords;
}

bool started = false;
bool reached_goal = false;
Point goalPos;
Quaternion goalOrientation;

void odomReceived(const Odometry::SharedPtr msg) {
	Pose pose = (msg->pose).pose;
	Point currPos = pose.position;
	Quaternion currOrientation = pose.orientation;
	if (!started) {
		started = true;
		Point initialPos = currPos;

		double a = 1.00;
		double b = .5;

		Point x_vec;
		x_vec.x = 1;
		x_vec.y = 0;
		x_vec.z = 0;
		Point x_rotated = q_rotate(currOrientation, x_vec);

		Point y_vec;
		y_vec.x = 0;
		y_vec.y = 1;
		y_vec.z = 0;
		Point y_rotated = q_rotate(currOrientation, y_vec);

		Point a_component = vec_multiply(a, x_rotated);
		Point b_component = vec_multiply(b, y_rotated);
		Point shift = vec_add(a_component, b_component);

		goalPos = vec_add(initialPos, shift);
		goalOrientation = currOrientation;

		RCLCPP_INFO(nodeh->get_logger(), pointToString(currPos).c_str());
		RCLCPP_INFO(nodeh->get_logger(), pointToString(goalPos).c_str());
	}

	Point relativeGoalPos = vec_subtract(goalPos, currPos);
	Quaternion robotInverse = q_inverse(currOrientation);
	Point goalPosRobot = q_rotate(robotInverse, relativeGoalPos);
	double dist = vec_magnitude(goalPosRobot);
	Twist currTwist;
	if (dist >= .10) {
		double theta = acos(goalPosRobot.x / dist);
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

		// RCLCPP_INFO(nodeh->get_logger(), "dist: %f", dist);
		// RCLCPP_INFO(nodeh->get_logger(), "theta: %f", theta);
	} else {
		currTwist.linear.x = 0;
		currTwist.angular.z = 0;
		if (!reached_goal) {
			RCLCPP_INFO(nodeh->get_logger(), "GOAL HAS BEEN REACHED");
			reached_goal = true;
		}
	}
	publisher->publish(currTwist);
}

int main(int argc, char **argv) {
	init(argc, argv);
	
	nodeh = Node::make_shared("goal_goer_guy");	
	auto sub = nodeh->create_subscription<Odometry>("/odom", 10, &odomReceived);
	publisher = nodeh->create_publisher<Twist>("/cmd_vel", 1); 
	RCLCPP_INFO(nodeh->get_logger(), "hello");
	
	spin(nodeh);
	shutdown();
	return 0;
}

float x_target = 1.00;
float y_target = .5;