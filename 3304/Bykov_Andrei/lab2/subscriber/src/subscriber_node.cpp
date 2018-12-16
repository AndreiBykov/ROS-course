#include <ros/ros.h>
#include "message/coordinates.h"
#include <stdlib.h>
#include <vector>

void callback_func(const message::coordinates& attack_point) {
	ROS_INFO_STREAM("attack point = ( " << (int) attack_point.x << " ; " << (int) attack_point.y << " )");

	std::vector<message::coordinates> defence_points;
	for(int i = 0; i < 3; i++) {
		message::coordinates defence_point;
		defence_point.x = rand() % 5 + 1;
		defence_point.y = rand() % 5 + 1;
		defence_points.push_back(defence_point);
		ROS_INFO_STREAM("defence point = ( " << (int) defence_point.x << " ; " << (int) defence_point.y << " )");
	}

	int probability = 5;
	
	for(int i = 0; i < defence_points.size(); i++) {
		message::coordinates defence_point = defence_points[i];
		if(defence_point.x == attack_point.x && defence_point.y == attack_point.y) {
			probability = 80;
		}
	}

	ROS_INFO_STREAM("Probability = " << probability);

	if( rand() % 100 + 1 <= probability) {
		ROS_INFO_STREAM("Attacking missile shot down");
	} else {
		ROS_INFO_STREAM("Attacking missile didn't shoot down");
	}
	ROS_INFO_STREAM("----------------------------------");		
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "subscriber");
	
	srand(time(0));

	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("heaven", 1000, &callback_func);

	ros::spin();
}
