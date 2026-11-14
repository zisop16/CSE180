#include <geometry_msgs/msg/pose.hpp>
#include <cmath>
using namespace geometry_msgs::msg;

double vec_magnitude(Point p) {
	return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

Point vec_add(Point a, Point b) {
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
	return a;
}

Point vec_negate(Point a) {
	a.x *= -1;
	a.y *= -1;
	a.z *= -1;
	return a;
}

Point vec_subtract(Point a, Point b) {
	return vec_add(a, vec_negate(b));
}

Point vec_multiply(double k, Point p) {
	p.x *= k;
	p.y *= k;
	p.z *= k;
	return p;
}

Quaternion q_inverse (Quaternion q) {
	q.x *= -1;
	q.y *= -1;
	q.z *= -1;
	return q;
}

Quaternion from_vec (Point p) {
	Quaternion out;
	out.w = 0;
	out.x = p.x;
	out.y = p.y;
	out.z = p.z;
	return out;
}

Point to_vec (Quaternion q) {
	Point p;
	p.x = q.x;
	p.y = q.y;
	p.z = q.z;
	return p;
}

Quaternion q_multiply (Quaternion q1, Quaternion q2) {
	double new_w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	double new_x = q1.y * q2.z - q2.y * q1.z + q1.w * q2.x + q2.w * q1.x;
	double new_y = q1.z * q2.x - q2.z * q1.x + q1.w * q2.y + q2.w * q1.y;
	double new_z = q1.x * q2.y - q1.y * q2.x + q1.w * q2.z + q1.z * q2.w;
	Quaternion out;
	out.w = new_w;
	out.x = new_x;
	out.y = new_y;
	out.z = new_z;
	return out;
}


Point q_rotate (Quaternion orientation, Point p) {
	Quaternion q1 = q_multiply(orientation, from_vec(p));
	Quaternion q2 = q_multiply(q1, q_inverse(orientation));
	return to_vec(q2);
}


