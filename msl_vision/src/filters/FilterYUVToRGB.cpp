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
#include "FilterYUVToRGB.h"

#define CLIP(X) ( (X) > 255 ? 255 : (X) < 0 ? 0 : (X) )

// YUV -> RGB
#define C(Y) ( (Y) - 16  )
#define D(U) ( (U) - 128 )
#define E(V) ( (V) - 128 )

#define YUV2R(Y, U, V) CLIP(( 298 * C(Y)              + 409 * E(V) + 128) >> 8)
#define YUV2G(Y, U, V) CLIP(( 298 * C(Y) - 100 * D(U) - 208 * E(V) + 128) >> 8)
#define YUV2B(Y, U, V) CLIP(( 298 * C(Y) + 516 * D(U)              + 128) >> 8)

// YCbCr -> RGB
#define CYCbCr2R(Y, Cb, Cr) CLIP( Y + ( 91881 * Cr >> 16 ) - 179 )
#define CYCbCr2G(Y, Cb, Cr) CLIP( Y - (( 22544 * Cb + 46793 * Cr ) >> 16) + 135)
#define CYCbCr2B(Y, Cb, Cr) CLIP( Y + (116129 * Cb >> 16 ) - 226 )

// RGB -> YUV
#define RGB2Y(R, G, B) CLIP(( (  66 * (R) + 129 * (G) +  25 * (B) + 128) >> 8) +  16)
#define RGB2U(R, G, B) CLIP(( ( -38 * (R) -  74 * (G) + 112 * (B) + 128) >> 8) + 128)
#define RGB2V(R, G, B) CLIP(( ( 112 * (R) -  94 * (G) -  18 * (B) + 128) >> 8) + 128)

#include <algorithm>

FilterYUVToRGB::FilterYUVToRGB(int width, int height):Filter(OF_RGB, width, height) {

	init();

}



FilterYUVToRGB::~FilterYUVToRGB(){

	cleanup();

}
		

unsigned char * FilterYUVToRGB::processBack(unsigned char * src, unsigned int imagesize)
{    
	unsigned char * tgt = outputBuffer;
	
	for (unsigned int i = 0; i < imagesize; i += 6)
	{
		 int r1  = src[i];
     		 int g1  = src[i+1];
     		 int b1  = src[i+2];
     		 int r2  = src[i+3];
     		 int g2  = src[i+4];
     		 int b2  = src[i+5];

     		 *(tgt++) = RGB2U(r1, g1, b1);
     		 *(tgt++) = RGB2Y(r1, g1, b1);
     		 *(tgt++) = RGB2V(r2, g2, b2);
     		 *(tgt++) = RGB2Y(r2, g2, b2);
	}

	return outputBuffer;
}


unsigned char * FilterYUVToRGB::process(unsigned char * src, unsigned int imagesize){
    
    unsigned char * tgt = outputBuffer;

    for (unsigned int i = 0; i < imagesize; i += 4) {
      int u  = src[i];
      int y0 = src[i+1];
      int v  = src[i+2];
      int y1 = src[i+3];

//      *(tgt++) = t_r[(y0<<8)|v];
//      *(tgt++) = t_g2[(y0<<8)|t_g1[(u<<8)|v]];
//      *(tgt++) = t_b[(y0<<8)|u];
//
//      *(tgt++) = t_r[(y1<<8)|v];
//      *(tgt++) = t_g2[(y1<<8)|t_g1[(u<<8)|v]];
//      *(tgt++) = t_b[(y1<<8)|u];

      *(tgt++) = YUV2R(y0, u, v);
      *(tgt++) = YUV2G(y0, u, v);
      *(tgt++) = YUV2B(y0, u, v);

      *(tgt++) = YUV2R(y1, u, v);
      *(tgt++) = YUV2G(y1, u, v);
      *(tgt++) = YUV2B(y1, u, v);
    }

    return outputBuffer;


}



void FilterYUVToRGB::init(){

    for (int i = 0; i < 256; i++) {
      for (int j = 0; j < 256; j++) {

        int r = i + (((j - 128) * 1434) / 2048);
        int b = i + (((j - 128) * 2078) / 2048);
        int g1 = (((i - 128) * 406) / 2048) + (((j - 128) * 595) / 2048);
        int g2 = i - j;

        t_r[(i << 8) | j] = std::min(std::max(0, r), 255);
        t_b[(i << 8) | j] = std::min(std::max(0, b), 255);
        t_g1[(i << 8) | j] = std::min(std::max(0, g1), 255);
        t_g2[(i << 8) | j] = std::min(std::max(0, g2), 255);
      }
    }

}


void FilterYUVToRGB::cleanup(){




}

