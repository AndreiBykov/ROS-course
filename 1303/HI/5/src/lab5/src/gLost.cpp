#include <ros/ros.h>
#include <visualization_msgs/Marker.h>  
#include "geometry_msgs/Point.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <string>
#include <cstring>

#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include <fstream>
#include <geometry_msgs/Pose2D.h>

#define _USE_MATH_DEFINES // for C++
#include <cmath>

using namespace std;

float px=-5 , py=6;
string name ;
bool found= false; 
float lost_angle, alpha;
float dx, dy;
const float incr = 0.0174532925; // 1* -> radian;
ros::ServiceClient srv_client ;
bool success_listen;
float mspeed=1.0;
float rspeed= 2*M_PI;

gazebo_msgs::GetModelState getSate(ros::NodeHandle & nodeh , std::string relativeEntityName , std::string modelName )
{
  gazebo_msgs::GetModelState getModelState ;
	
  srv_client = nodeh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state") ;
  getModelState.request.model_name = modelName ;
  getModelState.request.relative_entity_name = relativeEntityName ;
  srv_client.call(getModelState) ;
	
	return getModelState;
}

void sendXYToTF( ros::NodeHandle & nodeh, std::string relativeEntityName , std::string modelName  )
{
	
  static tf::TransformBroadcaster br;
  tf::Transform transform;
 
  //std::string modelName = (std::string)"gLost" ;
  //std::string relativeEntityName = (std::string)"world" ;

  gazebo_msgs::GetModelState getModelState ;
  geometry_msgs::Point pp ;
  geometry_msgs::Quaternion qq ;
	
  getModelState=getSate(nodeh , relativeEntityName , modelName );

  pp = getModelState.response.pose.position ;
  qq = getModelState.response.pose.orientation ;
        
  transform.setOrigin( tf::Vector3(pp.x, pp.y, pp.z) ) ;
  tf::Quaternion q(qq.x, qq.y, qq.z, qq.w) ;
  transform.setRotation(q) ;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), relativeEntityName , modelName )) ;
	
  //perform orientation angle	
  double roll, pitch, yaw;	
  tf::Matrix3x3 matr(q);
  matr.getRPY(roll, pitch, yaw);
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << " lost_angle: "<< lost_angle);
  lost_angle=yaw;
	if (lost_angle<0) {
		lost_angle+= (2*M_PI);
		if (fabs(lost_angle-2*M_PI)<0.01 ) lost_angle =0.0;
	}
	else{
		if (fabs(lost_angle-2*M_PI)<0.01 ) lost_angle =0.0;
		//else if(lost_angle > 2*M_PI) lost_angle= fmod (lost_angle , ( 2*M_PI ));
	}
	
  //perfom x, y
  px=pp.x;
  py=pp.y;
	
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "sendXYtoTF --> px: "<< px <<" py: "<< py <<" lost_angle: "<< lost_angle*180/M_PI<< "*");
}


tf::StampedTransform listenTF(string coord_center, string node_name, ros::NodeHandle & nodeh)
{
	static   tf::TransformListener listener;
	tf::StampedTransform transform;
	
	do
	{
		sendXYToTF(nodeh, "world", "gLost");
    	try{
	  		ros::Time now = ros::Time::now();	
	 		listener.waitForTransform(coord_center, node_name, now, ros::Duration(2.0) );
      		listener.lookupTransform(coord_center, node_name, ros::Time(0), transform);
			success_listen = true;
    	}
    	catch (tf::TransformException &ex) {
	 		ROS_INFO_STREAM("--Catch Error--");
      		ROS_ERROR("%s",ex.what());
      		ros::Duration(1.0).sleep();
      		success_listen =false;
    	}
		sendXYToTF(nodeh, "world", "gLost");
	}while(!success_listen);
	
	
	return transform;
}


void setFind(const std_msgs::String & msg)
{
	std::string data = msg.data;
	if(data.compare("back")==0) found =true;
		
}



void move(ros::Publisher &pb, float sp, ros::NodeHandle & nodeh ) // ок
{
	gazebo_msgs::GetModelState ms = getSate( nodeh , (std::string) "world" , (std::string) "gLost"  );
	
	gazebo_msgs::ModelState msg;
    msg.model_name = "gLost";
	msg.pose = ms.response.pose;
	msg.twist = ms.response.twist;
    msg.twist.linear.x=sp;
    pb.publish(msg);
	ros::spinOnce();
    sleep(2.0);
	
}


void turn(ros::Publisher &pb, float angle, ros::NodeHandle & nodeh ) //ок
{
	gazebo_msgs::GetModelState ms = getSate( nodeh , (std::string) "world" , (std::string) "gLost"  );
	
	
	gazebo_msgs::ModelState msg;
    msg.model_name = "gLost";
	msg.pose = ms.response.pose;
	msg.twist = ms.response.twist;
    msg.twist.angular.z =angle;
    pb.publish(msg);
    ros::spinOnce();
	sleep(2.0);
		
}


void turnToRescuer(ros::Publisher &pb, ros::NodeHandle & nodeh )
{

	ros::spinOnce();
	tf::StampedTransform transf = listenTF("gLost", "gRescuer", nodeh);
	dx= transf.getOrigin().x();
	dy= transf.getOrigin().y();
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "turnToRescuer -> dx: "<< dx <<" dy: "<< dy);
	
	if(dx>0 && dy>=0) alpha= atan(fabs(dy/dx));
	if(dx==0 && dy>0) alpha= M_PI/2;
	if(dx<0 && dy>=0) alpha= M_PI - atan(fabs(dy/dx));
	if(dx<0 && dy<=0) alpha= M_PI + atan(fabs(dy/dx));
	if(dx>0 && dy<=0) alpha= 2*M_PI - atan(fabs(dy/dx));
	if(dx==0 && dy<0) alpha= 3*M_PI/2;

	
	float dth=lost_angle-alpha;
	while(fabs(lost_angle-alpha)>(5*incr))
	{
		turn(pb, 2*M_PI, nodeh);
		ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "turnToRescuer -> alpha: "<< alpha <<" lost_angle: "<< lost_angle);
		sendXYToTF( nodeh, "world" , "gLost");
	}	
}



int getMoveDirection()
{
	if(alpha>0 && alpha<M_PI)
	{
		 return -1;
	}
	else
	{
		if(alpha>M_PI)
		{
			return 1;
		}
		else
		{
			if(alpha==0)
			{
				return -1;
			}
			else
			{
				if(alpha==M_PI) return 1;
			}
		}
	}
}



int main(int argc, char **argv) 
{
	
	ros::init(argc,argv,"gLost");
	ros::NodeHandle nh;
	
	if (argc != 2){ROS_ERROR("need name as argument"); return -1;}
    name = argv[1];
	ROS_INFO_STREAM("name: "<< name);
	

		srand(time(NULL));
	/*do
	{
	  px= rand()%20-10;
	  py= rand()%20-10;
	}while(fabs(px)<2 && fabs(py)<2);*/
	
	
	//service running ***********************************************************************
	ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot =  nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
 
    ifstream fin("/home/tony/.gazebo/models/pioneer3at/model.sdf");
 
    string model;
    string buf;
    while(!fin.eof()){
        getline(fin, buf);
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = "gLost";
    geometry_msgs::Pose pose;
	pose.position.x= px;
    pose.position.y= py;
	pose.position.z=0;
    srv.request.initial_pose = pose;
    add_robot.call(srv);
	//running end *****************************************************************************
	
	
	
	ros::Publisher pub1 = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/gLost/cmd_vel", 10);
	ros::Subscriber back = nh.subscribe("back",10,setFind);	
	
	
	ros::Rate rate(30);
	while(ros::ok())
	{
		sendXYToTF(nh, "world", "gLost");
		ros::spinOnce();
		

		if(found && ( fabs(px)>0.5 && fabs(py)>0.5  ) )
		{
			
			//turnToRescuer(pub, nh );
			//move(pub1, 1.0, nh);
		}
		
		ros::spinOnce();
		rate.sleep();
	}
	
	
	return 0;
}