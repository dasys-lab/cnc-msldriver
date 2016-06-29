/*
 * $Id: FilterYUVToGray.cpp 1531 2006-08-01 21:36:57Z phbaer $
 *
 *
 * Copyright 2005,2006 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */
#include "FilterRGBToGray.h"

#include <algorithm>

FilterRGBToGray::FilterRGBToGray(int width, int height):Filter(OF_GRAY, width, height){

	init();

}



FilterRGBToGray::~FilterRGBToGray(){

	cleanup();

}


unsigned char * FilterRGBToGray::process(unsigned char * src, unsigned int imagesize){

	unsigned char * tgt = outputBuffer;

	for(unsigned int i = 0; i < imagesize / 3; i++)
	{
		*tgt = 0.2989 * src[0] + 0.5870 * src[1] + 0.114 * src[2];
		tgt++;
		src += 3;
	}

	return outputBuffer;

}



void FilterRGBToGray::init(){


}


void FilterRGBToGray::cleanup(){


}

