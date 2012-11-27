
#include "stdafx.h"
#include "positionAdjust.h"


//calculate the rotation mtx rotates <x, y, z> to <0, y', z>, vec dimention is 3*1, RZ dimention is 3*3
void calcZAxisRotateMtx(boost::numeric::ublas::matrix<float> vec, boost::numeric::ublas::matrix<float> &RZ )
{
	float x,y;
	x=vec(0,0);
	y=vec(1,0);
	float dis=sqrtf((x*x)+(y*y));
	float cosT=y/dis;
	float sinT=x/dis; 
	RZ(0,0)=cosT; RZ(0,1)=-sinT;  RZ(0,2)=0;
	RZ(1,0)=sinT;  RZ(1,1)=cosT;  RZ(1,2)=0;
	RZ(2, 0)=0;      RZ(2,1)=0;        RZ(2,2)=1;
}

//calculate the transfer vec transfer <x, y, z> to <x, y, 0>, vec dimention is 3*1
void calcTranVector(boost::numeric::ublas::matrix<float> vec, boost::numeric::ublas::matrix<float> &transVec)
{
	float z=vec(2, 0);
	transVec(0,0)=0;
	transVec(1,0)=0;
	transVec(2,0)=-z;
}

//
void adjustCoord(std::vector<Vec> &pointsList, boost::numeric::ublas::matrix<float> &RZ,  boost::numeric::ublas::matrix<float> &transVec, const int idx)
{
	boost::numeric::ublas::matrix<float> vec(3,1);
	vec(0,0)=pointsList.at(idx).x;
	vec(1,0)=pointsList.at(idx).y;
     vec(2,0)=pointsList.at(idx).z;
	 boost::numeric::ublas::matrix<float> rotatedvec(3,1);
	 calcZAxisRotateMtx(vec, RZ);
	 calcTranVector(vec, transVec);
	 for(int i=0; i<pointsList.size(); i++)
	 {
		 if(i!=idx)

		 {
			 vec(0,0)=pointsList.at(i).x;
			 vec(1,0)=pointsList.at(i).y;
			 vec(2,0)=pointsList.at(i).z;
			 rotatedvec=prod(RZ, vec);
			 pointsList.at(i).x=rotatedvec(0,0)+transVec(0,0);
			 pointsList.at(i).y=rotatedvec(1,0)+transVec(1,0);
			 pointsList.at(i).z=rotatedvec(2,0)+transVec(2,0);
		 }

		 else
		 {
			 pointsList.at(i).x=0;
			 pointsList.at(i).z=0;
		 }


	 }

}

void adjustCoord(mrpt::slam::CSimplePointsMap &pointsList,boost::numeric::ublas::matrix<float> RZ, boost::numeric::ublas::matrix<float> tranVec )
{
	boost::numeric::ublas::matrix<float> temp(3,1);
	for(int i=0; i< pointsList.getPointsCount(); i++)
	{
		pointsList.getPoint(i, temp(0,0), temp(1,0), temp(2, 0));
		boost::numeric::ublas::matrix<float> rotated=prod(RZ, temp);
		pointsList.setPoint(i, rotated(0,0)+tranVec(0,0), rotated(1,0)+tranVec(1,0), rotated(2,0)+tranVec(2,0));
	}

}


