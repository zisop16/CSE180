#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
// #include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "parse_coords.hpp"

#include <navigation/navigation.hpp>


using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace rclcpp;
using namespace std;
using namespace sensor_msgs::msg;
using namespace tf2_ros;

shared_ptr<Publisher<Twist, std::allocator<void>>> publisher;
SyncParametersClient::SharedPtr rotationToleranceClient;
shared_ptr<Navigator> nav_node;
shared_ptr<Node> search_node;
Buffer* transform_buffer;

Point ZERO_POINT;
Vector3 ZERO_VEC;
Twist ZERO_TWIST;
Quaternion IDENTITY;

void print_coordinate(const std::tuple<double, double>& point) {
    RCLCPP_INFO(search_node->get_logger(), "(%f, %f)\n", std::get<0>(point), std::get<1>(point));
}

void print_coordinate(const Vector3 vec) {
    RCLCPP_INFO(search_node->get_logger(), "(%f, %f, %f)\n", vec.x, vec.y, vec.z);
}

void log_pose(const geometry_msgs::msg::Pose& pose) {
	Logger logger = search_node->get_logger();
	RCLCPP_INFO(logger, "Position: (%.3f, %.3f, %.3f)\nOrientation: (%.3f, %.3f, %.3f, %.3f)",
				
		pose.position.x, pose.position.y, pose.position.z, 		
		pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
			
	);
}

void setRotationTolerance(bool enabled) {
	double tol;
	if (enabled) {
		tol = .1;
	} else {
		tol = 2 * M_PI;
	}
	if (rotationToleranceClient->wait_for_service(std::chrono::milliseconds(10))) {
		auto results = rotationToleranceClient->set_parameters({Parameter("general_goal_checker.yaw_goal_tolerance", tol)});
	} else {
		RCLCPP_INFO(search_node->get_logger(), "Failed to set rotation tolerance");
	}
}

enum navState {
	STARTUP, MOVING_TO_POINT, SPINNING, FINISHED
};

navState curr_state = STARTUP;
vector<tuple<double, double>> target_list;
int curr_target_ind = 0;

void timerCallback() {
	TransformStamped startingTransform;
	if (curr_state == STARTUP) {
		try {
			startingTransform = transform_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
			curr_state = MOVING_TO_POINT;
			Pose init_pose;
			init_pose.orientation = IDENTITY;
			init_pose.position = ZERO_POINT;
			tf2::doTransform(init_pose, init_pose, startingTransform);
			nav_node->SetInitialPose(make_shared<Pose>(init_pose));
			nav_node->WaitUntilNav2Active();
			log_pose(init_pose);
		}
		catch (tf2::TransformException& e) {
			return;
		}
	}

	if (curr_target_ind == target_list.size()) {
		RCLCPP_INFO(search_node->get_logger(), "FINISHED NAVIGATION\n");
		curr_state = FINISHED;
		return;
	}
	
	if (nav_node->IsTaskComplete()) {
		if (curr_state == MOVING_TO_POINT) {
			tuple<double, double> target_coords = target_list[curr_target_ind];
			Pose::SharedPtr target = make_shared<Pose>();
			target->orientation = IDENTITY;
			target->position.x = get<0>(target_coords);
			target->position.y = get<1>(target_coords);
			target->position.z = 0;
			setRotationTolerance(false);
			nav_node->GoToPose(target);
			curr_target_ind++;
			log_pose(*target);
			curr_state = SPINNING;
		} else if (curr_state == SPINNING) {
			setRotationTolerance(true);
			nav_node->Spin(2*M_PI);
			curr_state = MOVING_TO_POINT;
		}
		
	} else {

	}
	
}

int main(int argc, char **argv) {
	init(argc, argv);
	IDENTITY.x = 0;
	IDENTITY.y = 0;
	IDENTITY.z = 0;
	IDENTITY.w = 1;
	ZERO_VEC.x = ZERO_VEC.y = ZERO_VEC.z = ZERO_POINT.x = ZERO_POINT.y = ZERO_POINT.z = 0;
	ZERO_TWIST.angular = ZERO_VEC;
	ZERO_TWIST.linear = ZERO_VEC;

	nav_node = std::make_shared<Navigator>(false, false);
	search_node = make_shared<Node>("the_searchist");
	rclcpp::Clock::SharedPtr the_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
	Buffer buffer(the_clock);
	TransformListener listener(buffer);
	transform_buffer = &buffer;
	
	

	RCLCPP_INFO(search_node->get_logger(), "Search Robot Online");
	target_list = load_coordinates("search/warehouse_coordinates.txt");

	auto wallTimer = search_node->create_wall_timer(.1s, &timerCallback);

	auto tol_node = std::make_shared<rclcpp::Node>("goal_tol_setter");
	rotationToleranceClient = std::make_shared<rclcpp::SyncParametersClient>(tol_node, "/controller_server");

	spin(search_node);
	shutdown();

	return 0;
}