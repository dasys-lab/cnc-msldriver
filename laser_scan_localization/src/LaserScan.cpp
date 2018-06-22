/*
 * Listener.cpp
 *
 *  Created on: May 4, 2018
 *      Author: Carpe Noctem
 */

#include "laser_scan_localization/LaserLocalization.h"
#include "LaserScan.h"
#include <SystemConfig.h>
#include <geometry_msgs/Point.h>
#include <math.h>


namespace msl
{

LaserScan::LaserScan() {
	supplementary::SystemConfig *sc = supplementary::SystemConfig::getInstance();

    this->goalWidth = (*sc)["LaserScanLocalization"]->get<double>("LaserScanLocalization", "goalWidth", NULL);

    ros::NodeHandle n;

    // Gibt verarbeiteten Informationen wieder aus
    this->out = n.advertise<laser_scan_localization::LaserLocalization>("laser_scan_localization", 1000);

    // Verarbeitet die Nachrichten von urg_node
    this->in = n.subscribe("scan", 2000, &LaserScan::processScan, (LaserScan*) this);
}

// Maximum bestimmen fuer den Vektor
int LaserScan::maxIntensityOfScan(const std::vector<float> intensities, int start, int end)
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

void LaserScan::sendLocalization(const sensor_msgs::LaserScan::ConstPtr &scan, int firstMaxIndex, int secondMaxIndex)
{
    laser_scan_localization::LaserLocalization msg;

    float c = scan->ranges[firstMaxIndex] * 1000;
    float b = scan->ranges[secondMaxIndex] * 1000;
    float a = goalWidth;

    geometry_msgs::Point msgPointOne;
    msgPointOne.x = (scan->ranges[firstMaxIndex]) * 1000;
    msgPointOne.y = 0;
    msgPointOne.z = 0;

    geometry_msgs::Point msgPointTwo;
    msgPointTwo.x = ((pow(b, 2) + pow(c, 2) - pow(a, 2)) / (2 * c));
    msgPointTwo.y = sqrt(pow(b, 2) - pow(msgPointTwo.x, 2));
    msgPointTwo.z = 0;

    msg.points.push_back(msgPointOne);
    msg.points.push_back(msgPointTwo);

    out.publish(msg);
}

void LaserScan::sendLocalizationV2(const sensor_msgs::LaserScan::ConstPtr &scan, int firstMaxIndex, int secondMaxIndex)
{
    laser_scan_localization::LaserLocalization msg;

    float c = scan->ranges[firstMaxIndex] * 1000;
    float b = scan->ranges[secondMaxIndex] * 1000;
    float a = goalWidth;
    float al = (scan->angle_increment * abs(firstMaxIndex - secondMaxIndex));

    geometry_msgs::Point msgPointOne;
    msgPointOne.x = (scan->ranges[firstMaxIndex]) * 1000;
    msgPointOne.y = 0;
    msgPointOne.z = 0;

    geometry_msgs::Point msgPointTwo;
    msgPointTwo.x = b * cos(al);
    msgPointTwo.y = b * sin(al);
    msgPointTwo.z = al;

    msg.points.push_back(msgPointOne);
    msg.points.push_back(msgPointTwo);

    out.publish(msg);
}

// Informationen zu dem Index ausgeben
void LaserScan::printInfo(const sensor_msgs::LaserScan::ConstPtr &scan, int &maxIndex)
{
    float angle = scan->angle_min + (scan->angle_increment * maxIndex);

    ROS_INFO("Index: %d", maxIndex);
    ROS_INFO("Intensity: %f", scan->intensities[maxIndex]);
    ROS_INFO("Range: %f", scan->ranges[maxIndex]);

    //	ROS_INFO("Angle min: %f", scan->angle_min);
    ROS_INFO("Angle: %f", angle);
    //	ROS_INFO("Angle max: %f", scan->angle_max);
}

void LaserScan::processScan(const sensor_msgs::LaserScan::ConstPtr &scan)
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

    if (scan->intensities.size() != 1081)
    {
        ROS_INFO("Error in SCAN: %d", scan->intensities.size());
        return;
    }

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

    if (firstMaxBorderLeft == -1 || firstMaxBorderRight == -1 || secondMaxBorderLeft == -1 || secondMaxBorderRight == -1)
    {
        // Bad scan!
        ROS_INFO("Invalid max interval!");

        ROS_INFO("g1: %d", firstMaxBorderLeft);
        ROS_INFO("g2: %d", firstMaxBorderRight);
        ROS_INFO("g3: %d", secondMaxBorderLeft);
        ROS_INFO("g4: %d", secondMaxBorderRight);
        return;
    }

    // Indexe mit Maximas bestimmen
    int firstMaxIndex = maxIntensityOfScan(scan->intensities, firstMaxBorderLeft, firstMaxBorderRight);
    int secondMaxIndex = maxIntensityOfScan(scan->intensities, secondMaxBorderLeft, secondMaxBorderRight);

    // Testweise Info ausgeben
    ROS_INFO("INFO FIRST MAX:");
    printInfo(scan, firstMaxIndex);

    ROS_INFO("INFO SECOND MAX:");
    printInfo(scan, secondMaxIndex);

    // Bestimme Positionen und versende Nachricht
    sendLocalization(scan, firstMaxIndex, secondMaxIndex);
    // sendLocalizationV2(scan, firstMaxIndex, secondMaxIndex);
}
}

