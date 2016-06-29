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

void FilterRGBToHSV::RGBtoHSV( float r, float g, float b, float *h, float *s, float *v )
{
	float min, max, delta;
	min = std::fmin( std::fmin(r, g), b );
	max = std::fmax( std::fmax(r, g), b );
	*v = max;				// v
	delta = max - min;

	if( max != 0 )
	{
		*s = delta / max;		// s
	}
	else
	{
		// r = g = b = 0		// s = 0, v is undefined
		*s = 0;
		*h = -1;
		return;
	}

	if( r == max )
	{
		*h = ( g - b ) / delta;		// between yellow & magenta
	}
	else if( g == max )
	{
		*h = 2 + ( b - r ) / delta;	// between cyan & yellow
	}
	else
	{
		*h = 4 + ( r - g ) / delta;	// between magenta & cyan
	}
	*h *= 60;				// degrees
	if( *h < 0 )
	{
		*h += 360;
	}
}

void FilterRGBToHSV::HSVtoRGB( float *r, float *g, float *b, float h, float s, float v )
{
	int i;
	float f, p, q, t;
	if( s == 0 ) {
		// achromatic (grey)
		*r = *g = *b = v;
		return;
	}
	h /= 60;			// sector 0 to 5
	i = floor( h );
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );
	switch( i ) {
		case 0:
			*r = v;
			*g = t;
			*b = p;
			break;
		case 1:
			*r = q;
			*g = v;
			*b = p;
			break;
		case 2:
			*r = p;
			*g = v;
			*b = t;
			break;
		case 3:
			*r = p;
			*g = q;
			*b = v;
			break;
		case 4:
			*r = t;
			*g = p;
			*b = v;
			break;
		default:		// case 5:
			*r = v;
			*g = p;
			*b = q;
			break;
	}
}
		

unsigned char * FilterRGBToHSV::process(unsigned char * src, unsigned int imagesize)
{
    unsigned char * tgt = outputBuffer;

    for (unsigned int i = 0; i < imagesize; i += 3)
    {
      float r = (float)(*src++) / 255.0;
      float g = (float)(*src++) / 255.0;
      float b = (float)(*src++) / 255.0;
      float h = 0.0;
      float s = 0.0;
      float v = 0.0;

      this->RGBtoHSV(r, g, b, &h, &s, &v);

      s = 1.0;
      v = 1.0;

      this->HSVtoRGB(&r, &g, &b, h, s, v);

      *(tgt++) = (unsigned char)(h * 255.0 / 360.0);
      *(tgt++) = (unsigned char)(s * 255.0);
      *(tgt++) = (unsigned char)(v * 255.0);
    }

    return outputBuffer;
}



void FilterRGBToHSV::init()
{

}


void FilterRGBToHSV::cleanup(){




}

