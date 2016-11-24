/*
#include "RosHelperCalib.h"
#include <iostream>
#include "SystemConfig.h"
using namespace msl_msgs;
using namespace msl_actuator_msgs;

MotionControl RosHelperCalib::calibOdo;
ros::Publisher RosHelperCalib::calibOdo_pub;
ros::Subscriber RosHelperCalib::motionControl_sub;
ros::Subscriber RosHelperCalib::visionControl_sub;
ros::Subscriber RosHelperCalib::planControl_sub;
ros::AsyncSpinner *RosHelperCalib::spinner;

RawOdometryInfoPtr RosHelperCalib::rawData12;
msl_sensor_msgs::SimulatorWorldModelDataPtr RosHelperCalib::modelData12;
alica_ros_proxy::PlanTreeInfoPtr RosHelperCalib::oldState12;
float RosHelperCalib::length;


void RosHelperCalib::initialize(int argc, char** argv) {


	ros::init(argc, argv,"Gonzales");

	ros::NodeHandle n;

	RosHelperCalib::visionControl_sub = n.subscribe("/WorldModel/SimulatorWorldModelData", 1, handleVisionControlMessage);
	RosHelperCalib::motionControl_sub = n.subscribe("/RawOdometry", 1, handleMotionControlMessage);
	RosHelperCalib::planControl_sub = n.subscribe("/AlicaEngine/PlanTreeInfo", 1, handlePlanControlMessage);
	RosHelperCalib::calibOdo_pub = n.advertise<MotionControl>("/MotionControl", 10);

	supplementary::SystemConfig::getInstance();
	sleep(0.1);

	std::cout<< "CalibDrive started"<< std::endl;
	//spinner = new ros::AsyncSpinner(1);
	//spinner->start();
	//ros::Rate loop_rate(1);

	ros::spin();

}


void RosHelperCalib::handleMotionControlMessage(const msl_actuator_msgs::RawOdometryInfoPtr message){
	    //std::cout<< "I heard: msl_actuator_msgs"<< std::endl;
	    //std::cout<< message->motion<< std::endl;
	    //std::cout<< message->position<< std::endl;
	    rawData12 = message;
		return;
	}

void RosHelperCalib::handleVisionControlMessage(const msl_sensor_msgs::SimulatorWorldModelDataPtr message){
	    //std::cout<< "I heard: msl_sensor_msgs"<< std::endl;
	    //std::cout<< message->worldModel.odometry<< std::endl;
	    modelData12 = message;

		return;
	}

void RosHelperCalib::handlePlanControlMessage(const alica_ros_proxy::PlanTreeInfoPtr message){
	//std::cout<< "I heard: plan_msg"<< std::endl;
	if(oldState12 != nullptr)
	{
		if (oldState12->stateIDs[0] != message->stateIDs[0]){
			if (message->stateIDs[0] == 1443003834928){
			sleep(1);
			}
			calcOdometry();
		}
	}
	oldState12 = message;

	return;
}

void RosHelperCalib::calcOdometry() {
	if (rawData12 != nullptr && modelData12 != nullptr){
		std::cout<<"Differenzen: "<< std::endl;
		std::cout<<"x: " << rawData12->position.x-modelData12->worldModel.odometry.position.x<< std::endl;
		std::cout<<"y: " << rawData12->position.y-modelData12->worldModel.odometry.position.y<< std::endl;

		length = length + sqrt(modelData12->worldModel.odometry.position.x*modelData12->worldModel.odometry.position.x+modelData12->worldModel.odometry.position.y*modelData12->worldModel.odometry.position.y);
		std::cout<<"Länge: "<< length <<std::endl;

		//deltax = rawData12->position.x-modelData12->worldModel.odometry.position.x
		//deltay = rawData12->position.y-modelData12->worldModel.odometry.position.y
		//a = (sqrt(deltax²+deltay²)/L)+1          L=U*n     U'=a*U
	}
}


void RosHelperCalib::sendOdometry() {
		//calibOdo.motion.rotation = 1.0;
		//calibOdo.motion.translation = 2.0;
		//calibOdo.motion.angle = 0.0;
		//calibOdo.senderID = supplementary::SystemConfig::getOwnRobotID();
		//calibOdo_pub.publish(calibOdo);
}
*/
