#ifndef TPS_H
#define TPS_H
#include "../external/include/boost/numeric/ublas/matrix.hpp"
#include "../external/include/ludecomposition.h"

#include "FileIO.h"

static double tps_base_func(double r);//TPS base func
void TPSconverse(LPTSTR densePointsPath, 
	LPTSTR densePointsTransferedPath, 
	boost::numeric::ublas::matrix<double> mtx_weight, 
	std::vector<Vec> controlPoints, 
	std::vector<Vec> *transferedPoints);//make dense TPS converse

void TPSconverse(mrpt::slam::CSimplePointsMap &sourcePoints,
	mrpt::slam::CSimplePointsMap &alignedSourcePoints, 
	mrpt::slam::CSimplePointsMap &transferedPoints, 
	boost::numeric::ublas::matrix<double> mtx_weight, 
	std::vector<Vec> landMarks);

void calc_tps_single(std::vector< Vec > source_control_points,
	std::vector< Vec > destiny_control_points,
	boost::numeric::ublas::matrix<double> *mtx_weight, 
	boost::numeric::ublas::matrix<double> *mtx_orig_k);

BOOL getKM(Vec sourcePoint, 
	std::vector<Vec> controlPoints, 
	boost::numeric::ublas::matrix<double> *KM);

#endif
