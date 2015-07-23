#include "RosHelperCalib.h"

ros::Publisher RosHelperCalib::calibOdo_pub;
ros::Subscriber RosHelperCalib::motionControl_sub;
ros::Subscriber RosHelperCalib::visionControl_sub;
ros::AsyncSpinner *RosHelperCalib::spinner;



void RosHelperCalib::initialize(int argc, char** argv) {


	ros::init(argc, argv,supplementary::SystemConfig::robotNodeName("Gonzales"));

	ros::NodeHandle n;

	RosHelperCalib::visionControl_sub = n.subscribe("/WorldModel/WorldModelData", 10, handleVisionControlMessage);
	RosHelperCalib::motionControl_sub = n.subscribe("/RawOdometry", 10, handleMotionControlMessage);

	RosHelperCalib::calibOdo_pub = n.advertise<msl_actuator_msgs::MotionControl>("/MotionControl", 10);

	spinner = new ros::AsyncSpinner(1);
	spinner->start();
}


void RosHelperCalib::handleMotionControlMessage(const msl_actuator_msgs::RawOdometryInfoPtr message){
		ROS_INFO("I heard: [%s]", message->position);
		ROS_INFO("I heard: [%s]", message->motion);
		return;
	}

void RosHelperCalib::handleVisionControlMessage(const msl_sensor_msgs::WorldModelDataPtr message){
		ROS_INFO("I heard: [%s]", message->odometry);
		return;
	}

void RosHelper::sendOdometry() {
	//calibOdo_pub.publish(calibOdo);
}

