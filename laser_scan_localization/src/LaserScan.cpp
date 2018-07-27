/*
 * Listener.cpp
 *
 *  Created on: May 4, 2018
 *      Author: Carpe Noctem
 */

#include "LaserScan.h"
#include <msl_sensor_msgs/LaserLocalization.h>
#include <SystemConfig.h>
#include <geometry_msgs/Point.h>
#include <math.h>


namespace msl
{

LaserScan::LaserScan() {
    ros::NodeHandle n;

    // Gibt verarbeiteten Informationen wieder aus
    this->out = n.advertise<msl_sensor_msgs::LaserLocalization>("laser_scan_localization", 1000);

    // Verarbeitet die Nachrichten von urg_node
    this->in = n.subscribe("scan", 2000, &LaserScan::processScan, (LaserScan*) this);
}

// Maximale Intensitaet zwischen start und end.
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

// Maximum ist der Index zwischen start und end.
int LaserScan::maxIntensityOfScan(int start, int end)
{
	return start + round((end - start) / 2);
}

void LaserScan::sendLocalization(const sensor_msgs::LaserScan::ConstPtr &scan, int firstMaxIndex, int secondMaxIndex)
{
    msl_sensor_msgs::LaserLocalization msg;

    // Linke Torpfosten
    float r2 = scan->ranges[secondMaxIndex] * 1000;
    float al2 = scan->angle_min + scan->angle_increment * secondMaxIndex;
    geometry_msgs::Point msgPointTwo;
    msgPointTwo.x = -r2 * cos(al2);
    msgPointTwo.y = -r2 * sin(al2);
    msgPointTwo.z = 0;
    msg.points.push_back(msgPointTwo);

    // Rechte Torpfosten
    float r1 = scan->ranges[firstMaxIndex] * 1000;
    float al1 = scan->angle_min + scan->angle_increment * firstMaxIndex;
    geometry_msgs::Point msgPointOne;
    msgPointOne.x = -r1 * cos(al1);
    msgPointOne.y = -r1 * sin(al1);
    msgPointOne.z = 0;
    msg.points.push_back(msgPointOne);

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
        // Invalider Scan: Es konnten nicht zwei Ausschlaege gefunden werden
        ROS_INFO("Invalid max interval!");

        ROS_INFO("g1: %d", firstMaxBorderLeft);
        ROS_INFO("g2: %d", firstMaxBorderRight);
        ROS_INFO("g3: %d", secondMaxBorderLeft);
        ROS_INFO("g4: %d", secondMaxBorderRight);
        return;
    }

    // Indexe mit Maximas bestimmen
    int firstMaxIndex = maxIntensityOfScan(firstMaxBorderLeft, firstMaxBorderRight);
    int secondMaxIndex = maxIntensityOfScan(secondMaxBorderLeft, secondMaxBorderRight);

    // Testweise Info ausgeben
    ROS_INFO("INFO SECOND MAX (LEFT):");
    printInfo(scan, secondMaxIndex);
    ROS_INFO("INFO FIRST MAX (RIGHT):");
    printInfo(scan, firstMaxIndex);

    // Bestimme Positionen und versende Nachricht
    sendLocalization(scan, firstMaxIndex, secondMaxIndex);
}
}

