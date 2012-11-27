
#ifndef POSITIONADJUST_H
#define POSITIONADJUST_H

#include <math.h>
#include <vector>

#include "../external/include/boost/numeric/ublas/matrix.hpp"
#include "../external/include/mrpt/slam/CSimplePointsMap.h"
#include "Vec.h"

void calcRotateMtx(boost::numeric::ublas::matrix<float> vec, boost::numeric::ublas::matrix<float> &RZ);
void calcTranVector(boost::numeric::ublas::matrix<float> vec, boost::numeric::ublas::matrix<float> &transVec);
void adjustCoord(std::vector<Vec> &pointsList,  boost::numeric::ublas::matrix<float> &RZ,  boost::numeric::ublas::matrix<float> &transVec, const int idx);
void adjustCoord(mrpt::slam::CSimplePointsMap &pointsList,boost::numeric::ublas::matrix<float> RZ, boost::numeric::ublas::matrix<float> tranVec );
#endif