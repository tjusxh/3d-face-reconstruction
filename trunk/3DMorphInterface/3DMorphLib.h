#ifndef MORPHLIB_H
#define MORPHLIB_H
#include <vector>

namespace morph
{
	typedef struct morphPoint 
	{
		double x, y, z;
	} ;
	extern "C"
	{
		_declspec(dllexport) void readDensePointsFromPLY(
			std::vector<morphPoint> &outputDensePoints,
			LPTSTR path);
		
		_declspec(dllexport) void readControlPointsFromPP(
			std::vector<morphPoint> &outputPoints,
			LPTSTR path);
		
		_declspec(dllexport) void modelRegistration(
			LPTSTR refPath, 
			LPTSTR refLandMarkPath, 
			LPTSTR transferedPath, 
			LPTSTR transLandMarkPath,
			LPTSTR outputFilePath);

		_declspec(dllexport) void loadAllModels(
			LPTSTR path, 
			std::vector<std::vector<morphPoint>*> *landMarkSets, 
			std::vector<std::vector<morphPoint>*> *densePointsSets);
		
		_declspec(dllexport) void TPStransform(
			const std::vector<morphPoint> &srcLandmarks, 
            const std::vector<morphPoint> &disLandmarks,
			const std::vector<morphPoint> *inputDensePoints, 
			std::vector<morphPoint> *outputDensePoints);
		
		_declspec(dllexport) void leastSquareSolve(
			std::vector<morphPoint> *disPoints,
			std::vector<std::vector<morphPoint> *> *factorPoints,
			double &scale,
			std::vector<double> *result);
	};
}

#endif