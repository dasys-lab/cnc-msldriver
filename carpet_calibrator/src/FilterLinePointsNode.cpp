/*
 * FilterLinePointsNode.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: Lab-PC5
 */

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <FilterLinePointsCalib.h>
#include "../helpers/ScanLineHelper.h"


using namespace std;

int main(int argc, char **argv) {

	supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

	int area = ((*sc)["Vision"])->get<short>("Vision", "ImageArea", NULL);


	ScanLineHelper scanLineHelper;
	FilterLinePointsCalib linePointsfilter(area);

	int size = area*area*sizeof(unsigned char);

	unsigned char* data = (unsigned char*)malloc(size);

	//string directory = sc->getConfigPath() + "CarpetCalibImage.raw";
	string directory = string(argv[1]);
	cout << "Loading File: " << directory << endl;


	FILE * logfile = fopen(directory.c_str(), "r");
	fread(data, sizeof(char), size, logfile);
	fclose(logfile);

	linePointsfilter.process(data, area,area,scanLineHelper);
	cout << "1111" << flush;

	return 0;
}


