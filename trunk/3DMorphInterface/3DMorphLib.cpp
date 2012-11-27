#include "stdafx.h"
#include "3DMorphLib.h"
#include "FileIO.h"
#include "TPS.h"

#include "../external/include/ap.h"
#include "../external/include/interpolation.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <math.h>

using namespace std;
using namespace morph;
using namespace boost::numeric::ublas;

extern "C"
{
	_declspec (dllexport) void morph::readDensePointsFromPLY(
		std::vector<morphPoint> &outputDensePoints,
		LPTSTR path)
	{
		//mrpt::slam::CSimplePointsMap points;
		std::vector<Vec> points;
		readPointsFromPLY(&points, path);
		for(int i=0; i<points.size(); i++)
		{
			morphPoint p;
			float x,y,z;
			//points.getPoint( i, x, y, z);
			x = points[i].x;
			y = points[i].y;
			z = points[i].z;
		     p.x=x;
			 p.y=y;
			 p.z=z;
			outputDensePoints.push_back(p);
		}
	}
	_declspec(dllexport) void morph::readControlPointsFromPP(
		std::vector<morphPoint> &outputPoints,
		LPTSTR path)
	{
		std::vector<Vec> Vecpoints;
		readControlPointsFromPP(&Vecpoints, path);
		for(int i=0;i<Vecpoints.size(); i++)
		{
			morphPoint mp;
			mp.x=Vecpoints.at(i).x;
			mp.y=Vecpoints.at(i).y;
			mp.z=Vecpoints.at(i).z;
			outputPoints.push_back(mp);
		}
	}

	_declspec(dllexport) void morph::loadAllModels(LPTSTR directoryPath, 
		std::vector<std::vector<morphPoint>*> *landMarkSets, 
		std::vector<std::vector<morphPoint>*> *densePointsSets)
	{
		for(int i=0; i<landMarkSets->size(); i++)
		{
			delete landMarkSets->at(i);
		}
		landMarkSets->clear();

		for(int i=0; i<densePointsSets->size();i++)
		{
			delete densePointsSets->at(i);
		}
		densePointsSets->clear();

		CString allFilePath;
		allFilePath.Append(directoryPath);
		allFilePath.Append(_T("*.pp"));
		CFileFind ppFinder;
		bool isAny=ppFinder.FindFile(allFilePath.GetBuffer(0));
		int num=0;
		while(isAny!=0)
		{
			isAny=ppFinder.FindNextFile();
			num++;
			std::vector<morphPoint> *vec=new std::vector<morphPoint>();
			readControlPointsFromPP((*vec), ppFinder.GetFilePath().GetBuffer(0));
			landMarkSets->push_back(vec);

			CString plyFilePath;
			plyFilePath.Append(directoryPath);
			plyFilePath.Append(_T("RegisteredFaces\\"));
			plyFilePath.Append(ppFinder.GetFileTitle());
			plyFilePath.Append(_T("_registered"));
			plyFilePath.Append(_T(".ply"));
			std::vector<morphPoint> *model=new std::vector<morphPoint>();
			readDensePointsFromPLY((*model),plyFilePath.GetBuffer(0));
			densePointsSets->push_back(model);
		}
	}


	_declspec(dllexport) void morph::TPStransform(const std::vector<morphPoint>& srcLandmarks, 
		const std::vector<morphPoint>& disLandmarks,
		const std::vector<morphPoint> *inputDensePoints, 
		std::vector<morphPoint> *outputDensePoints)
	{
		/* Calculate the para matrix */
		int num=srcLandmarks.size();
		std::vector<Vec> srcLandVec, disLandVec;
		for(int i=0; i<num; i++)
		{
			Vec vs,vd;
			vs.x=srcLandmarks.at(i).x;
			vs.y=srcLandmarks.at(i).y;
			vs.z=srcLandmarks.at(i).z;
			srcLandVec.push_back(vs);
			vd.x=disLandmarks.at(i).x;
			vd.y=disLandmarks.at(i).y;
			vd.z=disLandmarks.at(i).z;
			disLandVec.push_back(vd);
		}
		matrix<double> mtx_weight( num+4, 4);
		matrix<double> mtx_k(num, num);
		calc_tps_single(srcLandVec,disLandVec, &mtx_weight, &mtx_k);

          /* Do the actual transform */
		for(int i=0; i<inputDensePoints->size(); i++ )
		{
			double x,y,z;
			x=inputDensePoints->at(i).x;
			y=inputDensePoints->at(i).y;
			z=inputDensePoints->at(i).z;
			Vec sourcePoint(x,y,z);
			matrix<double> KM(1, num+4);
			getKM(sourcePoint, srcLandVec, &KM);//get KM matrix for the source point.
			matrix<double> transferedPoint(1,4);
			//record transfered points
			transferedPoint=prod( KM, mtx_weight);
			morphPoint temp;
			temp.x=transferedPoint(0,0); temp.y=transferedPoint(0,1); temp.z=transferedPoint(0,2);
			outputDensePoints->push_back(temp);
		}
	}


	_declspec(dllexport) void morph::leastSquareSolve(std::vector<morphPoint> *disPoints,
		std::vector<std::vector<morphPoint> *> *factorPoints,
		double &scale,
		std::vector<double> *result)
	{
		alglib::real_2d_array landmarksMtx;
		alglib::real_1d_array y;
		int n=disPoints->size()*2;
		int m=factorPoints->size();
		landmarksMtx.setlength(n, m);
		for(int i=0; i<m ;i++)
		{
			std::vector<morphPoint> *temp=factorPoints->at(i);
			for(int j=0; j<disPoints->size(); j++)
			{
				landmarksMtx(2*j,i)=temp->at(j).x;
				landmarksMtx(2*j+1,i)=temp->at(j).z;
			}
		}
		y.setlength(n);
		y.setlength(n);
		for(int i=0;i<disPoints->size();i++)
		{
			y(i*2)=disPoints->at(i).x;
			y(i*2+1)=disPoints->at(i).z;
		}
		alglib::real_1d_array c;
		c.setlength(m);
		int info;
		alglib::lsfitreport report;
		alglib::lsfitlinear(y,landmarksMtx,n,m,info,c,report);
		for(int i=0; i<m ;i++)
		{
			result->push_back(c(i));
		}
	}


	_declspec (dllexport) void morph::modelRegistration(LPTSTR refPath, 
		LPTSTR refLandMarkPath, 
		LPTSTR transferedPath, 
		LPTSTR transLandMarkPath,
		LPTSTR outputFilePath)
	{
		float correspondRatio, sumSqrDist;
		std::vector<morphPoint> refPoints, srcPoints, transPoints,refLandMarks, srcLandMarks;

		readDensePointsFromPLY(refPoints, refPath);
		readDensePointsFromPLY(srcPoints, transferedPath);
		readControlPointsFromPP( refLandMarks, refLandMarkPath);
		readControlPointsFromPP(srcLandMarks, transLandMarkPath);

		/*if nose point's x or z is not 0, then adjust the model*/
		if(srcLandMarks.at(63).x!=0||srcLandMarks.at(63).z!=0)
		{
	
			float x=srcLandMarks.at(63).x;
			float y=srcLandMarks.at(63).y;
			float z=srcLandMarks.at(63).z;
			float dis=sqrtf((x*x)+(y*y));
			float cosT=y/dis;
			float sinT=x/dis; 

			/*rotate matrix*/
			boost::numeric::ublas::matrix<float> RZ(3,3);
			RZ(0,0)=cosT; RZ(0,1)=-sinT;  RZ(0,2)=0;
			RZ(1,0)=sinT;  RZ(1,1)=cosT;  RZ(1,2)=0;
			RZ(2, 0)=0;      RZ(2,1)=0;        RZ(2,2)=1;

			/*rotate and transfer*/
			boost::numeric::ublas::matrix<float> vec(3,1);
			for(int i=0; i<srcLandMarks.size();i++)
			{
				vec(0,0)=srcLandMarks.at(i).x;
				vec(1,0)=srcLandMarks.at(i).y;
				vec(2,0)=srcLandMarks.at(i).z;
				boost::numeric::ublas::matrix<float> res=prod(RZ, vec);
				srcLandMarks.at(i).x=res(0,0);
				srcLandMarks.at(i).y=res(1,0);
				srcLandMarks.at(i).z=res(2,0)-z;//transfer along z axis
			}
			for(int i=0; i<srcPoints.size();i++)
			{
				vec(0,0)=srcPoints.at(i).x;
				vec(1,0)=srcPoints.at(i).y;
				vec(2,0)=srcPoints.at(i).z;
				boost::numeric::ublas::matrix<float> res=prod(RZ, vec);
				srcPoints.at(i).x=res(0,0);
				srcPoints.at(i).y=res(1,0);
				srcPoints.at(i).z=res(2,0)-z;
			}
			/*force nose points' x and z coord to 0*/
			srcLandMarks.at(63).x=0;
			srcLandMarks.at(63).z=0;
			/*write adjusted feature points to pp file*/
			std::vector<Vec> featurePoints;
			for(int i=0; i<srcLandMarks.size(); i++)
			{
				Vec v;
				v.x=srcLandMarks.at(i).x;
				v.y=srcLandMarks.at(i).y;
				v.z=srcLandMarks.at(i).z;
				featurePoints.push_back(v);
			}
			writeControlPointsToPP(&featurePoints, transLandMarkPath,refLandMarkPath);
			

		}
		TPStransform(srcLandMarks, refLandMarks, &srcPoints, &transPoints);

		mrpt::poses::CPose3D pose;
		mrpt::poses::CPoint3D point;
		mrpt::slam::TMatchingPairList pairList;
		mrpt::slam::CSimplePointsMap refPointsMap , transPointsMap, registeredPointsMap;
		for(int i=0; i<refPoints.size(); i++)
		{
			refPointsMap.insertPoint( refPoints.at(i).x, refPoints.at(i).y, refPoints.at(i).z);
			registeredPointsMap.insertPoint( refPoints.at(i).x, refPoints.at(i).y, refPoints.at(i).z);
		}


		for(int i=0; i<transPoints.size(); i++)
		{
			transPointsMap.insertPoint(transPoints.at(i).x, transPoints.at(i).y, transPoints.at(i).z);
		}

		transPointsMap.computeMatchingWith3D( &refPointsMap, pose, 500, 500, point, pairList, correspondRatio, &sumSqrDist);
		for(mrpt::slam::TMatchingPairList::iterator iter=pairList.begin(); iter!=pairList.end(); iter++)
		{
			unsigned int other_idx=(*iter).other_idx;
			unsigned int this_idx=(*iter).this_idx;
			registeredPointsMap.setPoint(other_idx, srcPoints.at(this_idx).x,srcPoints.at(this_idx).y, srcPoints.at(this_idx).z );
		}

		writePointsToPly(&registeredPointsMap, refPath, outputFilePath);

	}
};