// ASM.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "stasm.hpp"

int main(void)
{
	const char *sImage = "../data/test-image.jpg";

	RgbImage Img(sImage);       // read the image

	SHAPE StartShape;           // dummy arg for AsmSearch
	DET_PARAMS DetParams;       // dummy arg for AsmSearch
	double MeanTime;            // dummy arg for AsmSearch

	SHAPE Shape = AsmSearch(StartShape, DetParams, MeanTime, Img, sImage);

	if (Shape.nrows())          // successfully located landmarks?
	{
		DrawShape(Img, Shape);  // draw landmark shape on image
		CropImageToShape(Img, Shape);
		WriteBmp(Img, "search-results.bmp", VERBOSE);
		// Shape.print("Landmarks:\n"); // print the landmarks if you want
	}

	return 0;
}


