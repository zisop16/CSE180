#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
// #include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "quaternions.hpp"

using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace rclcpp;
using namespace std;
using namespace sensor_msgs::msg;

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
Point goalPos;
Quaternion goalOrientation;

int N = 5;
double L = .8 + .05 * N;
double D_stop = .4 + .02 * N;

// Points in the odometry frame
Point W1_odom;
Point W2_odom;
Point W3_odom;
Point W4_odom;
Point ZERO_POINT;
Vector3 ZERO_VEC;
Twist ZERO_TWIST;
bool botIsStopped = false;
int mostRecentTarget = 1;


void computePatrolPath(Pose initialPose) {
	Quaternion initialOrientation = initialPose.orientation;
	Point initialPosition = initialPose.position;
	Point W = ZERO_POINT;
	W.x = L;
	W1_odom = vec_add(q_rotate(initialOrientation, W), initialPosition);
	W.y = L;
	W2_odom = vec_add(q_rotate(initialOrientation, W), initialPosition);
	W.x = 0;
	W3_odom = vec_add(q_rotate(initialOrientation, W), initialPosition);
	W.y = 0;
	W4_odom = vec_add(q_rotate(initialOrientation, W), initialPosition);
	RCLCPP_INFO(nodeh->get_logger(), "W1: %s", pointToString(W1_odom).c_str());
	RCLCPP_INFO(nodeh->get_logger(), "W3: %s", pointToString(W3_odom).c_str());
}


Point targetPos(int index) {
	switch(index) {
		case 1:
			return W1_odom;
		case 2:
			return W2_odom;
		case 3:
			return W3_odom;
		case 4:
			return W4_odom;
		default:
			return ZERO_POINT;
	}
}

Point computeTarget(Pose currPose) {
	Point currPos = currPose.position;
	Point recentTargetPos = targetPos(mostRecentTarget);
	Point distVec = vec_subtract(recentTargetPos, currPos);
	double dist = vec_magnitude(distVec);
	const double REACHED_DIST = .10;
	if (dist < REACHED_DIST) {
		mostRecentTarget++;
		if (mostRecentTarget == 5) {mostRecentTarget = 1;}
		return targetPos(mostRecentTarget);
	} else {
		return recentTargetPos;
	}
}

Twist computeTwistTo(Pose currPose, Point pos) {
	double maxLinSpeed = .2;
	double linSpeedFactor = .5;
	double maxAngSpeed = .8;
	double angSpeedFactor = 1.0;

	Quaternion currOrientation = currPose.orientation;
	Point currPos = currPose.position;
	Point relPos = vec_subtract(pos, currPos);
	Quaternion invOrient = q_inverse(currOrientation);
	// Position of the target point in Robot's local space
	relPos = q_rotate(invOrient, relPos);
	double dist = vec_magnitude(relPos);
	double theta = acos(relPos.x / dist);
	if (relPos.y < 0) theta *= -1;
	// Theta is the z axis rotation of the target point relative to robot's local space
	double angularDist = abs(theta);
	double angularSign = theta >= 0 ? 1 : -1;
	double targetAngSpeed = min(angularDist * angSpeedFactor, maxAngSpeed);
	double angularZVel = targetAngSpeed * angularSign;

	double linearXSpeed = 0;
	if (abs(theta) < .3) {
		linearXSpeed = min(dist * linSpeedFactor, maxLinSpeed);
	}

	Twist calculatedTwist = ZERO_TWIST;
	calculatedTwist.angular.z = angularZVel;
	calculatedTwist.linear.x = linearXSpeed;
	return calculatedTwist;
}

void odomReceived(const Odometry::SharedPtr msg) {
	if (botIsStopped) {return;}
	Pose pose = (msg->pose).pose;
	if (!started) {
		computePatrolPath(pose);
		started = true;
	}
	Point targetPos = computeTarget(pose);
	Twist currTwist = computeTwistTo(pose, targetPos);
	publisher->publish(currTwist);
}



void scanReceived(const LaserScan::SharedPtr msg) {
	vector<float> ranges = msg->ranges;
	bool obstructed = false;
	for (int i = 0; i < ranges.size(); i++) {
		if (ranges[i] < D_stop) {
			obstructed = true;
			break;
		}
	}
	bool changedStopState = botIsStopped != obstructed;
	botIsStopped = obstructed;
	if (changedStopState) {
		if (botIsStopped) {
			publisher->publish(ZERO_TWIST);
			RCLCPP_INFO(nodeh->get_logger(), "OBSTRUCTION ENCOUNTERED. PATROL OPERATION PAUSING");
		}
		else {
			RCLCPP_INFO(nodeh->get_logger(), "OBSTRUCTION ELIMINATED. PATROL OPERATION RESUMING");
		}
	}
}

int main(int argc, char **argv) {
	init(argc, argv);

	ZERO_VEC.x = ZERO_VEC.y = ZERO_VEC.z = ZERO_POINT.x = ZERO_POINT.y = ZERO_POINT.z = 0;
	ZERO_TWIST.angular = ZERO_VEC;
	ZERO_TWIST.linear = ZERO_VEC;


	nodeh = Node::make_shared("the_patrollist");	
	auto odomSub = nodeh->create_subscription<Odometry>("/odom", 10, &odomReceived);
	auto scanSub = nodeh->create_subscription<LaserScan>("/scan", 10, &scanReceived);
	publisher = nodeh->create_publisher<Twist>("/cmd_vel", 1); 
	RCLCPP_INFO(nodeh->get_logger(), "Patrol Robot Online.");
	
	spin(nodeh);
	shutdown();
	return 0;
}