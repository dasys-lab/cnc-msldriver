#include <ros/ros.h>
#include <string>
#include <vector>
#include <sensor_msgs/LaserScan.h>


namespace msl
{
	class LaserScan
	{
	public:
		LaserScan();

	private:
		/**
		 * Ausgabe fuer die verarbeiteten Laser Scans
		 */
		ros::Publisher out;

		ros::Subscriber in;

		// Grenzen der Intervalle
		int firstMaxBorderLeft;
		int firstMaxBorderRight;
		int secondMaxBorderLeft;
		int secondMaxBorderRight;


		void processScan(const sensor_msgs::LaserScan::ConstPtr &scan);

                int maxIntensityOfScan(int start, int end);
		int maxIntensityOfScan(const std::vector<float> intensities, int start, int end);
		void sendLocalization(const sensor_msgs::LaserScan::ConstPtr &scan, int firstMaxIndex, int secondMaxIndex);
		void printInfo(const sensor_msgs::LaserScan::ConstPtr &scan, int &maxIndex);

	};

};
