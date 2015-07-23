#ifndef RosHelperCalib_h
#define RosHelperCalib_h

#include "ros/ros.h"
#include "msl_actuator_msgs/MotionControl.h"

class RosHelperCalib {
	public:
		static void initialize(int argc, char** argv);
		static msl_actuator_msgs::RawOdometryInfo calibOdo;

		static ros::Publisher calibOdo_pub;
		static ros::Subscriber motionControl_sub;
		static ros::Subscriber visionControl_sub;
		static ros::AsyncSpinner *spinner;

		static void handleMotionControlMessage(const msl_actuator_msgs::RawOdometryInfoPtr message);
		static void handleVisionControlMessage(const msl_sensor_msgs::WorldModelDataPtr message);
		static void sendOdometry();

};



#endif
