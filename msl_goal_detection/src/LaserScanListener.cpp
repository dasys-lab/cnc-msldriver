/*
 * LaserScanListener.cpp
 *
 *  Created on: Jan 27, 2017
 *      Author:  Lisa Martmann
 */

#include <LaserScanListener.h>
#include <SystemConfig.h>

namespace msl
{
	LaserScanListener::LaserScanListener()
	{
		readConfigParameters();
		publisher = n.advertise<msl_msgs::Pose2dStamped>(publisher_topic.c_str(), 10);
//		publisher_filtered_max = n.advertise<geometry_msgs::PointStamped>(filter_topic.c_str(), 10);
		sub = n.subscribe(scanner_topic.c_str(), 10, &LaserScanListener::onLaserScanReceived, (LaserScanListener*)this);

	}

	LaserScanListener::~LaserScanListener()
	{
	}

	void LaserScanListener::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		scanner_topic = (*sc)["GoalDetection"]->get<string>("GoalDetection.Topics.scanner_topic", NULL);
		publisher_topic = (*sc)["GoalDetection"]->get<string>("GoalDetection.Topics.publisher_topic", NULL);
		filter_topic = (*sc)["GoalDetection"]->get<string>("GoalDetection.Topics.filter_topic", NULL);
		frame_id = (*sc)["GoalDetection"]->get<string>("GoalDetection.frame_id", NULL);
		fileName = (*sc)["GoalDetection"]->get<string>("GoalDetection.Logging.fileName", NULL);

		back_width = (*sc)["GoalDetection"]->get<double>("GoalDetection.back_width", NULL);
		goal_depth = (*sc)["GoalDetection"]->get<double>("GoalDetection.goal_depth", NULL);
		back_width_tolerance = (*sc)["GoalDetection"]->get<double>("GoalDetection.back_width_tolerance", NULL);
		max_angle_distance = (*sc)["GoalDetection"]->get<double>("GoalDetection.max_angle_distance", NULL);
		view_area_angle = (*sc)["GoalDetection"]->get<double>("GoalDetection.view_area_angle", NULL);
		max_distance = (*sc)["GoalDetection"]->get<double>("GoalDetection.max_distance", NULL);
		min_distance = (*sc)["GoalDetection"]->get<double>("GoalDetection.min_distance", NULL);
		reduction_factor = (*sc)["GoalDetection"]->get<double>("GoalDetection.reduction_factor", NULL);

		loggingEnabled = (*sc)["GoalDetection"]->get<bool>("GoalDetection.Logging.loggingEnabled", NULL);
		probeNum = (*sc)["GoalDetection"]->get<int>("GoalDetection.Logging.probeNum", NULL);

		scanner_offset = tf::Vector3(0.2, 0.0, 0);
		z_axis = tf::Vector3(0, 0, 1);
		y_axis = tf::Vector3(0, 1, 0);

		itCounter = 0;
		timesLogged = 0;

		initLogging();

	}

	void LaserScanListener::onLaserScanReceived(sensor_msgs::LaserScanPtr msg)
	{

		itCounter++;

		if (loggingEnabled && itCounter % 100 == 0 && timesLogged < probeNum)
		{
			for (int i = 0; i < msg->ranges.size(); i++)
			{
				cout << "Logging ranges" << endl;
//				if (msg->ranges[i] < back_width * 1.2)
//				{
				log(rawLog, i, msg->ranges[i]);
//				}
			}
			timesLogged++;
			log(rawLog, -1, -1);
		}

		// reduce points to flatten the points by averaging some of them out
		vector<pair<int, double>> reduced = smoothen_points(msg);
		// cout << "all count: " << msg->ranges.size() << endl;
		cout << "reduced pairs: " << reduced.size() << endl;

		// find maximum values of these points
//		vector<pair<int, double>> maxima = find_maxima(msg);
		vector<pair<int, double>> maxima = find_maxima(reduced);
		cout << "Maximum count: " << maxima.size() << endl;
//

//		// filter out measurement errors
		vector<pair<int, double>> okay_points = filter_points(maxima, msg);

		cout << "okay_points count: " << okay_points.size() << endl;
//

//		// convert maximums to cartesian coordinates to find back plane candidates later
		vector<tf::Vector3> points = polar_to_cartesian(okay_points, msg);
		cout << "points count: " << points.size() << endl;

//		// find candidates for the goal corners by maximums
		vector<pair<tf::Vector3, tf::Vector3>> corner_candidates = find_back_candidates(points);

		if (corner_candidates.size() > 0)
		{
			cout << "cc count: " << corner_candidates.size() << endl;
			// sort candidates by their distance to the scanner.
			// the more farther away these two points are, the better
			std::sort(corner_candidates.begin(), corner_candidates.end(),
						[](pair<tf::Vector3, tf::Vector3> a, pair<tf::Vector3, tf::Vector3> b)
						{
							double a_len = a.first.length() * a.first.length() + a.second.length() * a.second.length();
							double b_len = b.first.length() * b.first.length() + b.second.length() * b.second.length();
							return a_len > b_len;
						});

			vector<msl_msgs::Pose2dStamped> positions;
			for (auto corner_pair : corner_candidates)
			{
				auto p1 = corner_pair.first; // corner 1
				auto p2 = corner_pair.second; // corner 2
				auto back_candidate = p1 - p2; // back plane vector

				tf::Vector3 goalie_center = tf::Vector3(-0.2, 0.0, 0);

				// get relative angle of the back plane (to the robot)
				double theta = y_axis.angle(back_candidate);
				// vector orthogonal to the back plane
				// has the length of the goal depth
				auto post_vector = back_candidate.rotate(z_axis, M_PI / 2).normalized() * goal_depth;

				auto half_back_candidate = back_candidate.normalized() * (back_candidate.length() / 2.0);

				// position of the (half) back plane as a vector
				// this is the center of the back plane
				auto back_center_absolute = p2 + half_back_candidate;

				// add the vector of the goal depth to the absolute back plane center
				auto scanner_center_offset = back_center_absolute + post_vector;

				// add our offset of the scanner position to the goal keeper center
				auto offset = scanner_center_offset + scanner_offset;

				cout << "(" << p1.getX() << " | " << p1.getY() << ")" << " -> " << "(" << p2.getX() << " | "
						<< p2.getY() << ")" << "\t" << "[" << back_candidate.length() << "]" << "\t" << "[" << theta
						<< ", " << (theta * 180 / M_PI) << "]" << "\t" << "(" << back_center_absolute.getX() << " | "
						<< back_center_absolute.getY() << ")" << "(" << offset.getX() << " | " << offset.getY() << ")"
						<< endl;

				if (loggingEnabled && itCounter % 100 == 0)
				{
					ofstream os(fileName + "General.log", std::ofstream::app);
					os << "(" << p1.getX() << " | " << p1.getY() << ")" << " -> " << "(" << p2.getX() << " | "
							<< p2.getY() << ")" << "\t" << "[" << back_candidate.length() << "]" << "\t" << "[" << theta
							<< ", " << (theta * 180 / M_PI) << "]" << "\t" << "(" << back_center_absolute.getX()
							<< " | " << back_center_absolute.getY() << ")" << "(" << offset.getX() << " | "
							<< offset.getY() << ")" << endl;
					os.close();
				}
				double scanner_center_offset_length = scanner_center_offset.length();

				// check if the calculated goal position is extremely far away.
				// If so, this is point extremely wrong.
				// uses the back plane width as a reference value.. just because. check if obsolete
				if (scanner_center_offset_length > back_width * 3)
				{
					continue;
				}

				auto leftGoalCornerEgo = p1 - goalie_center;
				auto rightGoalCornerEgo = p2 - goalie_center;

				auto leftGoalPostEgo = leftGoalCornerEgo + post_vector;
				auto rightGoalPostEgo = rightGoalCornerEgo + post_vector;

				//for some reason, this doesn't seem to work? TODO !!!
				auto actualGoalMidEgo = (leftGoalPostEgo + rightGoalPostEgo) / 2;

				// Add this goal center to the potential points list.
				msl_msgs::Pose2dStamped pose;

				//fill msg values (x1000 because m to mm)
				pose.pose.x = actualGoalMidEgo.getX() * 1000;
				pose.pose.y = actualGoalMidEgo.getY() * 1000;
				pose.leftGoalPost.x = leftGoalPostEgo.getX() * 1000;
				pose.leftGoalPost.y = leftGoalPostEgo.getY() * 1000;
				pose.rightGoalPost.x = rightGoalPostEgo.getX() * 1000;
				pose.rightGoalPost.y = rightGoalPostEgo.getY() * 1000;

				//TODO check this
				pose.pose.theta = theta;
				positions.push_back(pose);

				//TODO why
//				break;
			}

			logPositions(positionsLog, positions);

			if (positions.size() > 0)
			{
				// just take the first one and publish it.
				msl_msgs::Pose2dStamped final = positions[0];

				//favor a position where angle theta corresponds to x and y coordinates
				//for now just take avg position
				int num = positions.size();

				for (auto pos : positions)
				{
					final.pose.x += pos.pose.x;
					final.pose.y += pos.pose.y;
					final.leftGoalPost.x += pos.leftGoalPost.x;
					final.leftGoalPost.y += pos.leftGoalPost.y;
					final.rightGoalPost.x += pos.rightGoalPost.x;
					final.rightGoalPost.y += pos.rightGoalPost.y;
					final.pose.theta += pos.pose.theta;
				}

				final.pose.x /= num;
				final.pose.y /= num;
				final.leftGoalPost.x /= num;
				final.leftGoalPost.y /= num;
				final.rightGoalPost.x /= num;
				final.rightGoalPost.y /= num;
				final.pose.theta /= num;

				final.header.frame_id = frame_id;
				final.header.stamp = ros::Time::now();

				publisher.publish(final);

//				publish_message(publisher, pos.x, pos.y, pos.theta);
			}
		}

	}

	vector<pair<int, double>> LaserScanListener::smoothen_points(sensor_msgs::LaserScanPtr msg)
	{

		cout << "ranges: " << msg->ranges.size() << endl;

		//not working by itself because of index issues during polar to cartesian conversion
		vector<double> reduced(msg->ranges.size() / reduction_factor);

		for (size_t i = 0; i < reduced.size(); ++i)
		{
			double sum = 0;
			for (int j = 0; j < reduction_factor; ++j)
			{
				sum += msg->ranges[i * reduction_factor + j];
			}
			sum /= reduction_factor;
			reduced[i] = sum;
		}

		cout << "red: " << reduced.size() << endl;

		vector<pair<int, double>> closestToAvg;

		for (int k = 0; k < reduced.size(); k++)
		{
			pair<int, double> bestCandidate = make_pair(k, msg->ranges[k * reduction_factor]);

			for (int l = 0; l < reduction_factor; l++)
			{

				if (abs(msg->ranges[k * reduction_factor + l] - reduced.at(k)) < abs(bestCandidate.second - reduced.at(k)))
				{
					bestCandidate = make_pair(k, msg->ranges[k * reduction_factor + l]);
				}

			}

			closestToAvg.push_back(bestCandidate);
		}

		return closestToAvg;

	}

//	vector<pair<int, double> > LaserScanListener::find_maxima(sensor_msgs::LaserScanPtr msg)
//	{
//		vector<pair<int, double>> points_pairs(msg->ranges.size());
//		for (size_t x = 0; x < msg->ranges.size(); ++x)
//		{
//			if (!(msg->ranges[x] < min_distance || msg->ranges[x] > max_distance))
//			{
//				points_pairs[x] = make_pair(x, msg->ranges[x]);
//			}
//			else
//			{
//				//mark values to be discarded as negative
//				points_pairs[x] = make_pair(x, -1);
//			}
//		}
//
//		std::sort(points_pairs.begin(), points_pairs.end(), [](pair<int, double> left, pair<int, double> right)
//		{
//			return left.second > right.second;
//		});
//
//		vector<int> xValues;
//		for (auto point : points_pairs)
//		{
//			auto x = point.first;
//			auto y = point.second;
//			if (y > 0 && std::find(xValues.begin(), xValues.end(), x) == xValues.end())
//			{
//				if (satisfies_threshold(xValues, x))
//				{
//					xValues.push_back(x);
//				}
//			}
//		}
//		vector<pair<int, double>> result;
//		for (auto x : xValues)
//		{
//			//cout << "adding " << msg->ranges[x] << "at idx " << x << endl;
//			result.push_back(make_pair(x, msg->ranges[x]));
//		}
//
//		return result;
//	}

	vector<pair<int, double> > LaserScanListener::find_maxima(vector<pair<int, double>> values)
	{
		vector<pair<int, double>> points_pairs(values.size());
		for (size_t x = 0; x < values.size(); ++x)
		{
			if (!(values.at(x).second < min_distance || values.at(x).second > max_distance))
			{
				points_pairs[x] = make_pair(values.at(x).first, values.at(x).second); // obsolete??
			}
			else
			{
				//mark values to be discarded as negative
				points_pairs[x] = make_pair(values.at(x).first, -1);
			}
		}

		std::sort(points_pairs.begin(), points_pairs.end(), [](pair<int, double> left, pair<int, double> right)
		{
			return left.second > right.second;
		});

		vector<int> xValues;
		for (auto point : points_pairs)
		{
			auto x = point.first;
			auto y = point.second;
			if (y > 0 && std::find(xValues.begin(), xValues.end(), x) == xValues.end())
			{
				if (satisfies_threshold(xValues, x))
				{
//					cout << "new x value" << x << endl;
					xValues.push_back(x);
				}
			}
		}
		vector<pair<int, double>> result;
		for (auto x : xValues)
		{
//			cout << "adding " << values.at(x).second << "at idx " << values.at(x).first << endl;
			result.push_back(make_pair(values.at(x).first, values.at(x).second));
		}

		return result;
	}

	bool LaserScanListener::satisfies_threshold(vector<int> vec, int value)
	{
		for (auto x : vec)
		{
			if (idxTooClose(x, value))
			{
				return false;
			}
		}
		return true;
	}

	vector<pair<int, double>> LaserScanListener::filter_points(vector<pair<int, double>> polars,
																sensor_msgs::LaserScanPtr msg)
	{
		vector<pair<int, double>> dest;
		for (auto value : polars)
		{
			double angle = msg->angle_min + msg->angle_increment * value.first;
			double length = value.second;
			cout << "LEN " << length << endl;
			cout << "angle: " << angle << endl;
			//TODO shouldn't it be view_area_angle/2?
			if (!(length < min_distance || length > max_distance || angle > view_area_angle
					|| angle < -view_area_angle ))
			{
				dest.push_back(value);
			}
		}
		return dest;
	}

	/**
	 * @brief Converts polar coordinates to cartesian coordinates
	 *
	 * @param polars Vector of polar coordinates.
	 * @param start_angle The start angle.
	 * @param angle_increment The amount each angle increments per measurement.
	 * @return A vector of Vector3, where Vector3.{x|y} are cartesian coordinates. Vector3.z is intentionally left 0.
	 */
	vector<tf::Vector3> LaserScanListener::polar_to_cartesian(vector<pair<int, double>> polars,
																sensor_msgs::LaserScanPtr msg)
	{
		vector<tf::Vector3> cartesians(polars.size());

		for (size_t i = 0; i < polars.size(); ++i)
		{
			auto value = polars[i];

			double angle = msg->angle_min + msg->angle_increment * value.first;
			double length = value.second;

			double x = length * cos(angle);
			double y = length * sin(angle);

			cartesians[i] = tf::Vector3(x, y, 0);
		}
		return cartesians;
	}

	vector<pair<tf::Vector3, tf::Vector3>> LaserScanListener::find_back_candidates(vector<tf::Vector3> maximums)
	{
		vector<pair<tf::Vector3, tf::Vector3>> candidates;

		for (auto point : maximums)
		{

			//continue if pt is behind robot or right of robot
//			if (point.getX() < 0 || point.getY() < 0)
//			{
//				cout << "cont1" << endl;
//				continue;
//			}

			for (auto other : maximums)
			{
				if (other == point)
				{
					continue;
				}

				tf::Vector3 back_candidate = point - other;
				double distance = back_candidate.length();
//				bool is_ok = is_in_range(distance);
//				if (is_ok)
//				{
					auto pair = make_pair(point, other);
					candidates.push_back(pair);
//				}
			}
		}
		return candidates;
	}

	bool LaserScanListener::is_in_range(double value)
	{
		return (back_width - back_width_tolerance) <= value && value <= (back_width + back_width_tolerance);
	}

	bool LaserScanListener::idxTooClose(int compare_to, int value)
	{
		return (compare_to - max_angle_distance) <= value && value <= (compare_to + max_angle_distance);
	}

	void LaserScanListener::initLogging()
	{
		if (loggingEnabled)
		{
			rawLog = fopen((fileName + "Raw.log").c_str(), "a");
//			smoothLog = fopen((fileName + "Smooth.log").c_str(), "a");
			positionsLog = fopen((fileName + "Positions.log").c_str(), "a");
//			generalLog = fopen((fileName + "General.log").c_str(), "a");

		}
	}

	void LaserScanListener::log(FILE* fp, double x, double y)
	{
		if (loggingEnabled)
		{
			fprintf(fp, "%f\t%f\n", x, y);
		}
	}

	void LaserScanListener::logPositions(FILE* file, vector<msl_msgs::Pose2dStamped> positions)
	{
		if (loggingEnabled)
		{
			for (auto position : positions)
			{
				fprintf(file, "%f\t%f\t%f\t%f\t%f\t%f\t%f\n", position.pose.x, position.pose.y, position.leftGoalPost.x,
						position.leftGoalPost.y, position.rightGoalPost.x, position.rightGoalPost.y,
						position.pose.theta);
			}
		}
	}

}

