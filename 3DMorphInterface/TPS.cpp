#include "stdafx.h"
#include "TPS.h"
#include "FileIO.h"
#include <iostream>


using namespace  boost::numeric::ublas;
using namespace std;
double regularization = 0.0;
double bending_energy = 0.0;

static double tps_base_func(double r)
{
	if ( r == 0.0 )
		return 0.0;
	else
		return r*r * log(r);
}


/*
*  Calculate Thin Plate Spline (TPS) weights from
*  control points 
*/
void calc_tps_single(std::vector< Vec > source_control_points,std::vector< Vec > destiny_control_points,boost::numeric::ublas::matrix<double> *mtx_weight, boost::numeric::ublas::matrix<double> *mtx_orig_k)
{
	cout<< "calculating TPS mtx..."<<endl;
	// You We need at least 3 points to define a plane
	if ( source_control_points.size() < 3||destiny_control_points.size()!=source_control_points.size() )
		return;

	unsigned p = source_control_points.size();

	// Allocate the matrix and vector
	matrix<double> mtx_l(p+4, p+4);

	// Fill K (p x p, upper left of L) and calculate
	// mean edge length from control points
	//
	// K is symmetrical so we really have to
	// calculate only about half of the coefficients.
	double a = 0.0;
	for ( unsigned i=0; i<p; ++i )
	{
		for ( unsigned j=i+1; j<p; ++j )
		{
			Vec pt_i = source_control_points[i];
			Vec pt_j = source_control_points[j];
			double elen = (pt_i - pt_j).len();//distance between two points
			mtx_l(i,j) = mtx_l(j,i) =
				(*mtx_orig_k)(i,j) = (*mtx_orig_k)(j,i) =
				tps_base_func(elen);
			a += elen * 2; // same for upper & lower tri
		}
	}
	a /= (double)(p*p);

	// Fill the rest of L
	for ( unsigned i=0; i<p; ++i )
	{
		// diagonal: reqularization parameters (lambda * a^2)
		mtx_l(i,i) = (*mtx_orig_k)(i,i) =
			regularization * (a*a);

		// P (p x 4, upper right)
		mtx_l(i, p+0) = source_control_points[i].x;
		mtx_l(i, p+1) = source_control_points[i].y;
		mtx_l(i, p+2) = source_control_points[i].z;
		mtx_l(i,p+3)=1;

		// P transposed (4 x p, bottom left)
		mtx_l(p+0, i) = source_control_points[i].x;
		mtx_l(p+1, i) = source_control_points[i].y;
		mtx_l(p+2, i) = source_control_points[i].z;
		mtx_l(p+3,i)=1;
	}
	// O (4 x 4, lower right)
	for ( unsigned i=p; i<p+4; ++i )
		for ( unsigned j=p; j<p+4; ++j )
			mtx_l(i,j) = 0.0;


	// Fill the right hand vector V
	for ( unsigned i=0; i<p; ++i )
	{
		(*mtx_weight)(i, 0) = destiny_control_points[i].x;
		(*mtx_weight)(i, 1) = destiny_control_points[i].y;
		(*mtx_weight)(i, 2) = destiny_control_points[i].z;
		(*mtx_weight)(i, 3) =1;
	}
	for(unsigned i=0; i<4; i++)
	{
		for(unsigned j=0; j<4; j++)
		{
			(*mtx_weight)(p+i,j)=0;

		}
	}

	// Solve the linear system "inplace", results in mtx_weight.
	if (0 != TPS::LU_Solve(mtx_l, (*mtx_weight)))
	{
		puts( "Singular matrix! Aborting." );
		exit(1);
	}
}

void TPSconverse(mrpt::slam::CSimplePointsMap &sourcePoints,mrpt::slam::CSimplePointsMap &alignedSourcePoints, mrpt::slam::CSimplePointsMap &transferedPoints, matrix<double> mtx_weight, std::vector<Vec> landMarks)
{
	cout<< "performing TPS translation..."<<endl;
	for(int i=0; i<sourcePoints.getPointsCount(); i++)
	{
		float  x, y, z;
		sourcePoints.getPoint( i, x, y, z);
		Vec point( x, y, z);
	     matrix<double> KM(1, landMarks.size()+4);
		 getKM(point, landMarks, &KM);//get KM matrix for the source point.
		 matrix<double> transferedPoint( 1, 4);

		 //record TPS transfered points
		 transferedPoint=prod( KM, mtx_weight);
		 transferedPoints.insertPoint( transferedPoint(0, 0), transferedPoint(0, 1), transferedPoint(0, 2));

		 //record affine transfered source points
		 matrix<double> D(4, 4);
		 for(int j=0; j<4; j++)
		 {
			 for(int k=0; k<4; k++)
			 {
				 D(j, k)= mtx_weight(landMarks.size()+j, k);
			 }
		 }
		 matrix<double> M(1, 4);
		 for(int j=0; j<4; j++)
		 {
			 M(0, j)=KM(0, landMarks.size()+j);
		 }
		 matrix<double> alignedSourcePoint(1, 4);
		 alignedSourcePoint=prod(M, D);
		 alignedSourcePoints.insertPoint(alignedSourcePoint(0, 0), alignedSourcePoint(0, 1), alignedSourcePoint(0, 2));
	}

}
//read points one by one from PLY file
// TPS translate and output them 
//into a new file
void TPSconverse(LPTSTR densePointsPath, LPTSTR densePointsTransferedPath, matrix<double> mtx_weight, std::vector<Vec> controlPoints,  std::vector<Vec> *transferedPointsSet)
{
	cout<< "performing TPS translation..."<<endl;

	ifstream inputFile(densePointsPath);
	ofstream outputFile(densePointsTransferedPath);
	if(inputFile.is_open())
	{
		int status=READING_HEADER;
		int pointsNum=0;
		int count=0;
		matrix<double> *points=NULL;
		while(!inputFile.eof())
		{
			char line[1000];
			inputFile.getline(line,1000);
			string s(line);

			switch(status)
			{

				//just copy header.
			case READING_HEADER:
				{
					outputFile<<line<<endl;
					if(s.find("element vertex")!=-1)
					{
						pointsNum=atol(s.substr(15,s.length()-15).c_str());
					}
					else if(s.find("end_header")!=-1)
					{
						status=READING_VERTEX;
					}
					break;
				}

				//if reading vertex right now, then using TPS convert the vertex coordinates. 
			case READING_VERTEX:
				{
					//eat up all empty lines
					if(s.size()==0&&count==0)
					{
						outputFile<<endl;
					}
					else if(count<pointsNum)
					{
						count++;
						double coord[3];
						getXYZPLY(s,coord);
						Vec sourcePoint(coord[0],coord[1],coord[2]);
						matrix<double> KM(1, controlPoints.size()+4);
						getKM(sourcePoint, controlPoints, &KM);//get KM matrix for the source point.
						matrix<double> transferedPoint(1,4);
						//record transfered points
						transferedPoint=prod(KM,mtx_weight);
						Vec p(transferedPoint(0,0),transferedPoint(0,1), transferedPoint(0,2));
						transferedPointsSet->push_back(p);
						outputFile<<transferedPoint(0,0)<<" "<<transferedPoint(0,1)<<" "<<transferedPoint(0,2)<<endl;
					}
					else if(count==pointsNum)
					{
						outputFile<<s<<endl;
						status=READING_POLY;
					}
					break;					 
				}

				//just copy the polys;
			case READING_POLY:
				{
					outputFile<<line<<endl;
					break;
				}
			}
		}
	}
}


//get matrix [KM] 
BOOL getKM(Vec sourcePoint,std::vector<Vec> controlPoints, matrix<double> *KM)
{
	int cpNum=controlPoints.size();
	//check whether KM have the right size.
	if(KM->size1()!=1||KM->size2()!=cpNum+4)
	{
		return 0;
	}

	//get K first
	int i=0;
	for(std::vector<Vec>::iterator it=controlPoints.begin();it!=controlPoints.end();it++)
	{
		double r=((*it)-sourcePoint).len();
		(*KM)(0,i)=tps_base_func(r);
		i++;
	}
	//get M
	for(int j=0;j<4;j++)
	{
		switch(j)
		{
		case 0:
			{
				(*KM)(0,j+cpNum)=sourcePoint.x;
				break;
			}
		case 1:
			{
				(*KM)(0,j+cpNum)=sourcePoint.y;
				break;
			}
		case 2:
			{
				(*KM)(0,j+cpNum)=sourcePoint.z;
				break;
			}
		case 3:
			{
				(*KM)(0,j+cpNum)=1;
				break;
			}
		}
	}
	return 1;
}