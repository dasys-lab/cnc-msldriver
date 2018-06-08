#include <ros/ros.h>
#include <string>
#include "msl_msgs/PositionInfo.h"
#include "geometry_msgs/PointStamped.h"


using namespace std;

namespace msl
{
	class LaserScan
	{
	public:
		LaserScan();


	private:
		int firstMaxBorderLeft;
		int firstMaxBorderRight;
		int secondMaxBorderLeft;
		int secondMaxBorderRight;



	}
