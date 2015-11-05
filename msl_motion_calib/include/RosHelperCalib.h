#ifndef RosHelperCalib_h
#define RosHelperCalib_h

#include "ros/ros.h"
#include "msl_actuator_msgs/MotionControl.h"
#include "msl_actuator_msgs/RawOdometryInfo.h"
#include "msl_sensor_msgs/VisionControl.h"
#include "msl_sensor_msgs/WorldModelData.h"
#include "msl_sensor_msgs/SimulatorWorldModelData.h"
#include "alica_ros_proxy/PlanTreeInfo.h"

class RosHelperCalib {
	public:
		static void initialize(int argc, char** argv);
		static msl_actuator_msgs::MotionControl calibOdo;

		static ros::Publisher calibOdo_pub;
		static ros::Subscriber motionControl_sub;
		static ros::Subscriber visionControl_sub;
		static ros::Subscriber planControl_sub;
		static ros::AsyncSpinner *spinner;

		static msl_sensor_msgs::SimulatorWorldModelDataPtr modelData12;
		static msl_actuator_msgs::RawOdometryInfoPtr rawData12;
		static alica_ros_proxy::PlanTreeInfoPtr oldState12;
		static float length;

		static void handleMotionControlMessage(const msl_actuator_msgs::RawOdometryInfoPtr message);
		static void handleVisionControlMessage(const msl_sensor_msgs::SimulatorWorldModelDataPtr message);
		static void handlePlanControlMessage(const alica_ros_proxy::PlanTreeInfoPtr message);
		static void calcOdometry();
		static void sendOdometry();

};



#endif
