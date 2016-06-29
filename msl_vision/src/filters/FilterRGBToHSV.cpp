/*
 * $Id: FilterYUVToRGB.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "FilterRGBToHSV.h"

#include <algorithm>

FilterRGBToHSV::FilterRGBToHSV(int width, int height):Filter(OF_RGB, width, height) {

	init();

}



FilterRGBToHSV::~FilterRGBToHSV(){

	cleanup();

}
		

unsigned char * FilterRGBToHSV::process(unsigned char * src, unsigned int imagesize)
{
    unsigned char * tgt = outputBuffer;

    for (unsigned int i = 0; i < imagesize; i++)
    {
      int r = *src++ / 255;
      int g = *src++ / 255;
      int b = *src++ / 255;
      int h = 0;
      int s = 0;
      int v = 0;

      int max = std::max(std::max(r,g),b);
      int min = std::min(std::min(r,g),b);

      if(min == max)
      {
    	  h = 0;
      }
      else if(max == r)
      {
    	  h = 60 * (g-b) / (max-min);
      }
      else if(max == g)
      {
    	  h = 60 * (b-r) / (max-min);
      }
      else if(max == b)
      {
    	  h = 60 * (r-g) / (max-min);
      }

      if(h < 0)
      {
    	  h += 360;
      }

      if(max == 0)
      {
    	  s = 0;
      }
      else
      {
    	  s = (max - min) / max;
      }

      v = max;

      *(tgt++) = h;
//      *(tgt++) = s;
//      *(tgt++) = v;
    }

    return outputBuffer;
}



void FilterRGBToHSV::init()
{

}


void FilterRGBToHSV::cleanup(){




}

