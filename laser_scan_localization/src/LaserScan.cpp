/*
 * Listener.cpp
 *
 *  Created on: May 4, 2018
 *      Author: Carpe Noctem
 */

#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include <vector>
// #include "laser_scan_localization/LaserLocalization.h"
#include <std_msgs/String.h>

// Ausgabe fuer die verarbeiteten Laser Scans
ros::Publisher out;

// Maximum bestimmen fuer den Vektor
int maxIntensityOfScan(const std::vector<float> &intensities, int start, int end)
{
    float maxIntensity = 0.0;
    int maxIndex = 0.0;
    for (unsigned int i = start; i < end; i = i + 1)
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

    int maxIndex = maxIntensityOfScan(scan->intensities, 0, scan->intensities.size());
    // printInfo(scan, maxIndex);


    //
    // Bestimmen von zwei Bereichen mit hohen Ausschlaegen
    //

    // true -> aktueller index liegt in einem Bereich mit hohen Ausschlag
    bool testIntervall = false;

    // Grenzen fuer die Ausschlaege. Wir gehen davon aus das es IMMER 2 Bereiche mit hoher Intensitaet gibt
    int firstMaxBorderLeft = -1;
    int firstMaxBorderRight = -1;
    int secondMaxBorderLeft = -1;
    int secondMaxBorderRight = -1;

    // Grenzwert zum erkennen der Bereiche mit hohen Ausschlaegen
    const float MAX_INTENSITY_THRESHOLD = 10000;

    // Grenzen bestimmen
    for (unsigned int i = 0; i < scan->intensities.size(); i = i + 1)
    {
        float x = scan->intensities[i];
        if (x > MAX_INTENSITY_THRESHOLD && testIntervall == false)
        {
            testIntervall = true;
            if (firstMaxBorderLeft == -1)
            {
                firstMaxBorderLeft = i;
            }
            else
            {
                secondMaxBorderLeft = i;
            }
        }
        if (x < MAX_INTENSITY_THRESHOLD && testIntervall == true)
        {
            testIntervall = false;
            if (firstMaxBorderRight == -1)
            {
                firstMaxBorderRight = i;
            }
            else
            {
                secondMaxBorderRight = i;
            }
        }
    }

    // Testweise Grenzen ausgeben
    ROS_INFO("g1: %d", firstMaxBorderLeft);
	ROS_INFO("g2: %d", firstMaxBorderRight);
	ROS_INFO("g3: %d", secondMaxBorderLeft);
	ROS_INFO("g4: %d", secondMaxBorderRight);

    // Indexe mit Maximas bestimmen
    int firstMaxIndex = maxIntensityOfScan(scan->intensities, firstMaxBorderLeft, firstMaxBorderRight);
    int secondMaxIndex = maxIntensityOfScan(scan->intensities, secondMaxBorderLeft, secondMaxBorderRight);

    // Testweise Info ausgeben
    ROS_INFO("INFO FIRST MAX:");
    printInfo(scan, firstMaxIndex);

    ROS_INFO("INFO SECOND MAX:");
    printInfo(scan, secondMaxIndex);

    // TODO eigene Nachricht zusammenbauen
//    laser_scan_localization::LaserLocalization msg;
    std_msgs::String msg;
	msg.data = "TODO OUTPUT";
	out.publish(msg);

    // TODO unsere Position bestimmen (Trigonometrie) - ALICA?
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "myscan");
    ros::NodeHandle n;

    // Gibt verarbeiteten Informationen wieder aus
    out = n.advertise<std_msgs::String>("laser_scan_localization", 1000);

    // Verarbeitet die Nachrichten von urg_node
    ros::Subscriber in = n.subscribe("scan", 2000, processScan);

    ros::spin();

    return 0;
}
