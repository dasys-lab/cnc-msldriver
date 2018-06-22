/*
 * Listener.cpp
 *
 *  Created on: Juni 22, 2018
 *      Author: Carpe Noctem
 */

#include <ros/ros.h>
#include <LaserScan.h>
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{

    std::string node_name = "myscan";
    ros::init(argc, argv, node_name);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    msl::LaserScan *lsl = new msl::LaserScan();

    while (ros::ok())
    {
        std::chrono::milliseconds dura(500);
        std::this_thread::sleep_for(dura);
    }

    spinner.stop();

    delete lsl;

    return 0;
}
