#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <cmath>  

using namespace std;

class Searcher {
private:
	//base info.
	string name;
	ros::NodeHandle &nh;
	int id;
	double x, y;
	double angle;
	double goalX, goalY;
	double gapX, gapY, gap;
	double gapAngle;

	string nameWalker;

	//for visualization.
	ros::Publisher pub;
	gazebo_msgs::ModelState robotState;


	void move(const std::string& targetFrame, const std::string& sourceFrame) {
		std::cout << "The robot searcher (" << name << ") begin to search (" << sourceFrame << ")" << std::endl;

		tf::TransformListener listener;
		tf::StampedTransform transform;

		int iter = 0;
		ros::Rate rate(100);
		while (nh.ok()) {
			try {
					listener.lookupTransform(targetFrame, sourceFrame, ros::Time(0), transform);
				}
				catch (tf::TransformException &e) {
					ROS_ERROR("%s", e.what());
					broadcastPosition();
					ros::Duration(1.0).sleep();
					continue;
				}

			if (iter == 0) {
				iter = 1000;

				goalX += transform.getOrigin().x();
				goalY += transform.getOrigin().y();

				double dx = goalX - x;
				double dy = goalY - y;
				gapX = dx / iter;
				gapY = dy / iter;
				gapAngle = (-angle + atan2(dy, dx)) / iter;

				if (transform.getOrigin().length() < gap)
					return;
			}

			if (transform.getOrigin().length() > gap) {
				x += gapX;
				y += gapY;
				angle += gapAngle;
			} //else stopping, because not to collide!
			iter--;

			broadcastPosition();
			repaint(); //repaint state.

			ros::spinOnce();
			rate.sleep();
		}
	}

	void cry(const std::string& msg) {
		std_msgs::String s;
		s.data = msg;
	  	ros::Publisher pubMsg = nh.advertise<std_msgs::String>("/voice", 100);
	 
	 	cout << "Cry message: (" << msg << ")" << endl;
	  	ros::Rate loop_rate(1);
	  	for (int i = 0; i < 3; i++) {
	        pubMsg.publish(s);
	        loop_rate.sleep();
	    }
	}

	void broadcastPosition() {
		//for broadcast.
		static tf::TransformBroadcaster br;
		tf::Transform transform;

		// cout << "Searcher (" << name <<") went to x:" << x << " y:" << y << endl;

		transform.setOrigin(tf::Vector3(x, y, 0.0));
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
	}

	void repaint() {
		robotState.pose.position.x = x;
	    robotState.pose.position.y = y;
	    robotState.pose.orientation.z = sin(angle / 2);
	    robotState.pose.orientation.w = cos(angle / 2);
	    pub.publish(robotState);
	}


public:
	Searcher(ros::NodeHandle &nh, string name, string nameWalker) : nh(nh), name(name), nameWalker(nameWalker) {
		id = 2;
		x = y = 0;
		gapX = gapY = 0;
		gap = 1.5;
		angle = 0;

		pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
	}

	void search() {
		move(name, nameWalker);
		cry(name);	
	}

	void comeback() {
		gap = 0.5;
		move(name, "world");
		cry("stop");
	}

	void initVisualisation() {
		cout << "init visualization" << endl;

		ros::service::waitForService("gazebo/spawn_sdf_model");
	    ros::ServiceClient add_robot = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	    gazebo_msgs::SpawnModel srv;
	 
	
	    // ifstream fin("/home/user/.gazebo/models/pioneer3at/model.sdf");
	    // ifstream fin("/home/user/.gazebo/models/pioneer2dx/model.sdf");
	    ifstream fin("/home/vlad/.gazebo/models/pioneer3at/model.sdf");

	    
 

		string model;
	    string buf;
	    while(!fin.eof()){
	        getline(fin, buf);
	        model += buf + "\n";
	    }
	    srv.request.model_xml = model;
	    srv.request.model_name = name;
	    geometry_msgs::Pose pose;
	    srv.request.initial_pose = pose;
	    add_robot.call(srv);

	    //state...
	    robotState.model_name = name;
	    robotState.pose.position.x = 0.0;
		robotState.pose.position.y = 0.0;
		robotState.pose.position.z = 0.0;
		robotState.pose.orientation.z = 0.0;
		robotState.pose.orientation.w = 0.0;

		broadcastPosition();
		repaint();
		sleep(3);
	}

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_searcher");
	sleep(5);


	if (argc < 2) {
		ROS_ERROR("Not specified robot's name as an argument!");
		return -1;
	}

	if (argc < 3) {
		ROS_ERROR("Not specified walker robot's name as an argument!");
		return -1;
	}

	ros::NodeHandle nh;
	Searcher searcher(nh, argv[1], argv[2]);
	searcher.initVisualisation();
	searcher.search();
	searcher.comeback();

	return 0;
}
