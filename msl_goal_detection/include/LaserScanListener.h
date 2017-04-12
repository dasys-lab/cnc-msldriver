/*
 * LaserScanListener.h
 *
 *  Created on: Jan 27, 2017
 *      Author:  Lisa Martmann
 */

#ifndef INCLUDE_LASERSCANLISTENER_H_
#define INCLUDE_LASERSCANLISTENER_H_

#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <memory>
#include <vector>
#include <iostream>
#include "msl_msgs/PositionInfo.h"
#include "geometry_msgs/PointStamped.h"
#include "msl_msgs/Pose2dStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_datatypes.h>

using namespace std;

namespace msl
{
	class LaserScanListener
	{
	public:
		LaserScanListener();
		virtual ~LaserScanListener();

		void onLaserScanReceived(sensor_msgs::LaserScanPtr msg);

	private:
		string scanner_topic;
		string publisher_topic;
		string filter_topic;
		string fileName;
		ros::NodeHandle n;
		ros::Publisher publisher;
		//TODO check if obsolete
//		ros::Publisher publisher_filtered_max;
		ros::Subscriber sub;
		double back_width;
		double goal_depth;
		double back_width_tolerance;
		//distance between 2 laser scan indices
		double max_angle_distance;
		double view_area_angle;
		double min_distance;
		double max_distance;
		double reduction_factor;

		int probeNum;
		int itCounter;
		int timesLogged;

		FILE* rawLog;
		FILE* smoothLog;
		FILE* positionsLog;
		FILE* generalLog;

		tf::Vector3 scanner_offset;
		tf::Vector3 z_axis;
		tf::Vector3 y_axis;

		string frame_id;

		bool loggingEnabled;

		void readConfigParameters();
		vector<pair<int,double>> smoothen_points(sensor_msgs::LaserScanPtr msg);
//		vector<pair<int, double>> find_maxima(sensor_msgs::LaserScanPtr msg);
		vector<pair<int, double>> find_maxima(vector<pair<int,double>> values);
		bool satisfies_threshold(vector<int> vec, int value);
		bool is_in_range(double value);
		bool idxTooClose(int compare_to, int value);
		vector<pair<int, double>> filter_points(vector<pair<int, double>> polars, sensor_msgs::LaserScanPtr msg);
		vector<tf::Vector3> polar_to_cartesian(vector<pair<int, double>> polars, sensor_msgs::LaserScanPtr msg);
		vector<pair<tf::Vector3, tf::Vector3>> find_back_candidates(vector<tf::Vector3> maximums);
		double rad_to_degree(double rad);
		void initLogging();
		void log(FILE* file,double x, double y);
		void logGeneralInfo();
		void logPositions(FILE* file, vector<msl_msgs::Pose2dStamped> positions);
	};
}

#endif /* INCLUDE_LASERSCANLISTENER_H_ */
