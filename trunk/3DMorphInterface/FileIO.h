#ifndef  FILEIO_H
#define FILEIO_H
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "Vec.h"
#include <windows.h>

#include "../GLMlib/glm.h"

#include "../external/include/mrpt/slam/CSimplePointsMap.h"

#define  READING_HEADER 0
#define  READING_VERTEX 1
#define  READING_POLY     2
void readControlPointsFromPP(std::vector<Vec>* points, LPTSTR path);//read control points from XML file
void writeControlPointsToPP(std::vector<Vec>*points, LPTSTR targetPath, LPTSTR refPath);//write control points to XML file
void readPointsFromPLY(mrpt::slam::CSimplePointsMap *points, LPTSTR path);//read dense points from PLY file
void readPointsFromPLY(std::vector<Vec> *points, LPTSTR densePointsPath);
void getXYZPP(std::string s, double* coord );//get xyz coordinate from string line in PP file.
void getXYZPLY(std::string s, double* coord);//get xyz coordinates from string line in PLY file.
void getTriangleFacesPLY(std::string s,unsigned int *idx);
bool writePointsToPly(mrpt::slam::CSimplePointsMap *points, LPTSTR referenceFilePath, LPTSTR outPutFilePath);
GLMmodel * readGLMfromPLY(char *PLYpath);
#endif