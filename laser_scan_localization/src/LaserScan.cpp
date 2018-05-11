/*
 * Listener.cpp
 *
 *  Created on: May 4, 2018
 *      Author: Carpe Noctem
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

void processScan(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	// Wir wissen:
	// scan->intensities.size() = 1081
	// scan->range.size() = 1081
	// maxIntensity liegt ungefähr zwischen 9.500 - 17.000

	float maxIntensity = 0.0;
	int maxIndex = 0.0;
	for( unsigned int i = 0; i < scan->intensities.size(); i = i + 1 ) {
		float x = scan->intensities[i];
		//ROS_INFO("Element: %lu - [%f]", i, x);
		if(maxIntensity < x) {
			maxIntensity = x;
			maxIndex = i;
		}
	}

	ROS_INFO("Index: %d", maxIndex);
	ROS_INFO("Intensity: %f", maxIntensity);
	ROS_INFO("Range: %f", scan->ranges[maxIndex]);

	// TODO Winkel zu den Punkt mit maximaler Intensität ausrechnen
	// TODO Zweiten hellsten Punkt (bzw. Höhepunkt aus "Graphen") bestimmen
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "myscan");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("scan", 2000, processScan);
	ros::spin();

	return 0;
}

