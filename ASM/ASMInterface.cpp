#include "ASMInterface.h"
#include "../external/include/stasm.hpp"

static SHAPE StartShape;           // dummy arg for AsmSearch
static DET_PARAMS DetParams;       // dummy arg for AsmSearch
static double MeanTime;            // dummy arg for AsmSearch
static int nInitializedModels=0;
static ASM_MODEL Models[2];
static const char sConfFile0[]="D:/Master Thesis/Code/ASM/stasm2.4/data/model-1.conf";
static const char sConfFile1[]="D:/Master Thesis/Code/ASM/stasm2.4/data/model-2.conf";
static const char sDataDir[]="D:/Master Thesis/Code/ASM/stasm2.4/data";

extern "C"
{
	_declspec(dllexport) void ASM::initASM()
	{
		nInitializedModels= nInitAsmModels(Models, sConfFile0,  sConfFile1);
	}

	_declspec(dllexport) void ASM::getASMPoints(const char *imgName, std::vector<ASMPoint> * pointsSet, int &width, int &height)
	{
		if(nInitializedModels==0)
		{
			std::cout<<"not initialized yet";
		}
		RgbImage Img(imgName); 
		SHAPE Shape = AsmSearch2(StartShape,DetParams, MeanTime, nInitializedModels, Models, Img, imgName,false,sConfFile0, sConfFile1,sDataDir);
		if (Shape.nrows())          // successfully located landmarks?
		{
			for(int i=0; i<Shape.size1();i++)
			{
				ASMPoint point;
				/* convert openGL coordinates (down left) */
				point.x=Shape.getElem(i,0)+Img.width/2;
				point.y=Shape.getElem(i,1)+Img.height/2;
				point.idx=i;
				pointsSet->push_back(point);
			}

		}
		width=Img.width;
		height=Img.height;
	}

};
