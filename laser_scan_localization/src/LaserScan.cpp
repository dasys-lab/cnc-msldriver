/*
 * Listener.cpp
 *
 *  Created on: May 4, 2018
 *      Author: Carpe Noctem
 */

#include "sensor_msgs/LaserScan.h"
#include "laser_scan_localization/LaserLocalization.h"
#include "ros/ros.h"
#include <LaserScan.h>
#include <SystemConfig.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <std_msgs/String.h>
#include <vector>

// Ausgabe fuer die verarbeiteten Laser Scans
ros::Publisher out;
double goalWidth;

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

    //    laser_scan_localization::LaserLocalization msg;
    laser_scan_localization::LaserLocalization msg;
    // msg.data = "TODO OUTPUT";
    geometry_msgs::Point msgPointOne;
    msgPointOne.x = (scan->ranges[firstMaxIndex]) * 1000;
    msgPointOne.y = 0;
    msgPointOne.z = 0;

    geometry_msgs::Point msgPointTwo;

    float c = scan->ranges[firstMaxIndex] * 1000;
    float b = scan->ranges[secondMaxIndex] * 1000;
    float a = goalWidth;

    msgPointTwo.x = ((pow(b, 2) + pow(c, 2) - pow(a, 2)) / (2 * c));
    msgPointTwo.y = sqrt(pow(b, 2) - pow(msgPointTwo.x, 2));
    msgPointTwo.z = 0;

    msg.points.push_back(msgPointOne);
    msg.points.push_back(msgPointTwo);

    out.publish(msg);

    laser_scan_localization::LaserLocalization msg2;
    // msg.data = "TODO OUTPUT";
    geometry_msgs::Point msgPointOne2;
    msgPointOne2.x = (scan->ranges[firstMaxIndex]) * 1000;
    msgPointOne2.y = 0;
    msgPointOne2.z = 0;

    geometry_msgs::Point msgPointTwo2;

    float al = (scan->angle_increment * abs(firstMaxIndex - secondMaxIndex));

    msgPointTwo2.x = b * cos(al);
    msgPointTwo2.y = b * sin(al);
    msgPointTwo2.z = al;

    msg2.points.push_back(msgPointOne2);
    msg2.points.push_back(msgPointTwo2);

    out.publish(msg2);

    // TODO unsere Position bestimmen (Trigonometrie) - ALICA?
    // standpunkt torfosten , robotor 0,0
    // system config
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "myscan");
    ros::NodeHandle n;

    supplementary::SystemConfig *sc = supplementary::SystemConfig::getInstance();
    goalWidth = (*sc)["LaserScanLocalization"]->get<double>("LaserScanLocalization", "goalWidth", NULL);

    // Gibt verarbeiteten Informationen wieder aus
    out = n.advertise<laser_scan_localization::LaserLocalization>("laser_scan_localization", 1000);

    // Verarbeitet die Nachrichten von urg_node
    ros::Subscriber in = n.subscribe("scan", 2000, processScan);

    ros::spin();

    return 0;
}
