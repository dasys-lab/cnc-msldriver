/*
 * Listener.cpp
 *
 *  Created on: May 4, 2018
 *      Author: Carpe Noctem
 */

#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include <vector>

// Maximum bestimmen fuer den Vektor
int maxIntensityOfScan(const std::vector<float> &intensities)
{
    float maxIntensity = 0.0;
    int maxIndex = 0.0;
    for (unsigned int i = 0; i < intensities.size(); i = i + 1)
    {
        float x = intensities[i];
        // ROS_INFO("Element: %lu - [%f]", i, x);
        if (maxIntensity < x)
        {
            maxIntensity = x;
            maxIndex = i;
        }
    }
    return maxIndex;
}

// Informationen zu dem Index ausgeben
void printInfo(const sensor_msgs::LaserScan::ConstPtr &scan, int &maxIndex)
{
    float angle = scan->angle_min + (scan->angle_increment * maxIndex);

    ROS_INFO("Index: %d", maxIndex);
    ROS_INFO("Intensity: %f", scan->intensities[maxIndex]);
    ROS_INFO("Range: %f", scan->ranges[maxIndex]);

    //	ROS_INFO("Angle min: %f", scan->angle_min);
    ROS_INFO("Angle: %f", angle);
    //	ROS_INFO("Angle max: %f", scan->angle_max);
}

void processScan(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    // Info:
    // http://wiki.ros.org/urg_node
    // http://wiki.ros.org/hokuyo_node/Tutorials
    // http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html

    // Wir wissen:
    // scan->intensities.size() = 1081
    // scan->range.size() = 1081
    // maxIntensity liegt ungefähr zwischen 9.500 - 17.000
    // angle_min = -2.356194
    // angle_max = +2.356194

    int maxIndex = maxIntensityOfScan(scan->intensities);
    // printInfo(scan, maxIndex);


    // Bestimmen von zwei Bereichen mit hohen Ausschlaegen
    bool testIntervall = false;
    int g1 = -1;
    int g2 = -1;
    int g3 = -1;
    int g4 = -1;
    const float GRENZWERT = 10000;
    for (unsigned int i = 0; i < scan->intensities.size(); i = i + 1)
    {
        float x = scan->intensities[i];
        if (x > GRENZWERT && testIntervall == false)
        {
            testIntervall = true;
            if (g1 == -1)
            {
                g1 = i;
            }
            else
            {
                g3 = i;
            }
        }
        if (x < GRENZWERT && testIntervall == true)
        {
            testIntervall = false;
            if (g2 == -1)
            {
                g2 = i;
            }
            else
            {
                g4 = i;
            }
        }
    }

    ROS_INFO("g1: %d", g1);
    ROS_INFO("g2: %d", g2);
    ROS_INFO("g3: %d", g3);
    ROS_INFO("g4: %d", g4);

    // TODO Fuer jeden Bereich das Maximum bestimmen und daraus unsere Position bestimmen (Trigonometrie)
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "myscan");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("scan", 2000, processScan);
    ros::spin();

    return 0;
}
