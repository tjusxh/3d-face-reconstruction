// useTestLib.cpp : Defines the entry point for the console application.

#include "stdafx.h"
#include <string>
#include <cstring>
#include <vector>
#include <cmath>

#include "../external/include/ap.h"

#include "register.h"
#include "FileIO.h"
#include "TPS.h"
#include "positionAdjust.h"


//#include <lsfit.h>


using namespace boost::numeric::ublas;
using namespace std;
using namespace std;
using namespace mrpt;
//using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::slam;

//function declaration.

/*
matrix<double> readDensePoints(LPTSTR path);//read dense points from ply file
void writeDensePoints(const matrix<double> *changedDensePoints, LPTSTR path); //write dense points to a ply file
*/

/* 
* input directory path, adjust all
* landmarks and ply files in that
* directory.
*/

void adjustAllMeshes(LPTSTR directoryPath)
{
	CString allFilePath;
	allFilePath.Append(directoryPath);
	allFilePath.Append(_T("\\*.pp"));
	CFileFind ppFinder;
	bool isAny=ppFinder.FindFile(allFilePath);
	while(isAny!=0)
	{
		//adjust pp points

		isAny=ppFinder.FindNextFile();
		CString ppFileName=ppFinder.GetFileTitle();
		CString last8Char=ppFileName.Right(8);
		if(last8Char.Compare(_T("adjusted"))!=0)
		{
			CString ppFilePath=ppFinder.GetFilePath();
			cout<<"processing: "<< ppFilePath<<endl;
			std::vector<Vec> landmarks;
			readControlPointsFromPP(&landmarks,ppFilePath.GetBuffer(0));
			matrix<float> RZ(3,3);
			matrix<float> transVec(3,1);
			adjustCoord(landmarks, RZ,transVec, 63);

			//write pp points
			CString adjustedPpPath;
			adjustedPpPath.Append(directoryPath);
			adjustedPpPath.Append(_T("\\"));
			adjustedPpPath.Append(ppFinder.GetFileTitle());
			adjustedPpPath.Append(_T("_adjusted.pp"));
			writeControlPointsToPP(&landmarks, adjustedPpPath.GetBuffer(0), ppFilePath.GetBuffer(0) );

			//adjust ply points
			CString plyFilePath;
			plyFilePath.Append(directoryPath);
			plyFilePath.Append(_T("\\"));
			plyFilePath.Append(ppFinder.GetFileTitle());
			plyFilePath.Append(_T(".ply"));
			cout<<"processing: "<<plyFilePath<<endl;
			mrpt::slam::CSimplePointsMap vertexes;
			readPointsFromPLY(&vertexes, plyFilePath.GetBuffer(0));
			adjustCoord(vertexes, RZ, transVec);

			//write ply points
			CString adjustedPLYPath;
			adjustedPLYPath.Append(directoryPath);
			adjustedPLYPath.Append(_T("\\"));
			adjustedPLYPath.Append(ppFinder.GetFileTitle());
			adjustedPLYPath.Append(_T("_adjusted.ply"));
			writePointsToPly(&vertexes, plyFilePath.GetBuffer(0),adjustedPLYPath.GetBuffer(0));
		}
	}

}






void read2DAllLandmarks(LPTSTR directoryPath, alglib::real_2d_array &landmarksMtx)
{
	CString allFilePath;
	allFilePath.Append(directoryPath);
	allFilePath.Append(_T("\\*.pp"));
	CFileFind ppFinder;
	bool isAny=ppFinder.FindFile(allFilePath.GetBuffer(0));
	int num=0;
	std::vector<std::vector<Vec>*> landmarksSet;
	while(isAny!=0)
	{
		isAny=ppFinder.FindNextFile();
		num++;
		std::vector<Vec> *vec=new std::vector<Vec>();
		readControlPointsFromPP(vec, ppFinder.GetFilePath().GetBuffer(0));
		landmarksSet.push_back(vec);
	}
	landmarksMtx.setlength(128, num);
	for(int i=0; i<num; i++)
	{
		std::vector<Vec> *temp= landmarksSet.at(i);
		//only record first 64 points!!! 
		for(int j=0; j<64; j++)
		{
			landmarksMtx(2*j,i)=temp->at(j).x;
			landmarksMtx(2*j+1,i)=temp->at(j).z;
		}
		delete temp;
	}
}


/*


int _tmain(int argc, _TCHAR* argv[])
{


	ap::real_2d_array landmarksMtx;
	read2DAllLandmarks(_T("D:\\Master Thesis\\PickedPointsFaces"), landmarksMtx);
	int n=landmarksMtx.gethighbound(1);
	int m=landmarksMtx.gethighbound(2);
	std::vector<Vec> targetlandmark;

	readControlPointsFromPP(&targetlandmark,_T("D:\\Master Thesis\\PickedPointsFaces\\M0009A1EnCtrimR0_adjusted.pp") );
	ap::real_1d_array y;
	y.setlength(n);
	for(int i=0;i<64;i++)
	{
		y(i*2)=targetlandmark.at(i).x;
		y(i*2+1)=targetlandmark.at(i).z;
	}
	ap::real_1d_array c;
	c.setlength(m);
	int info;
     lsfitreport report;
	lsfitlinear(y,landmarksMtx,n,m,info,c,report);
	
	for(int i=0; i<m;i++)
	{
		cout<<c(i)<<",";
	}
	 return 0;
*/
/*
	//calculate the weights matrix of TPS
	LPTSTR srcLandmarksPath=_T("D:\\Master Thesis\\PLY Faces\\Reference_picked_points.pp");
	LPTSTR targetLandmarksPath=_T("D:\\Master Thesis\\PLY Faces\\Reference_picked_points_adjusted.pp");
	std::vector<Vec> sourceLandmarks, destinyLandmarks;
	readControlPointsFromPP(&sourceLandmarks, srcLandmarksPath);
	matrix<float> RZ(3,3);
	matrix<float> transMTX(3,1);
	adjustCoord(sourceLandmarks, RZ, transMTX,63);
	writeControlPointsToPP(&sourceLandmarks,targetLandmarksPath,srcLandmarksPath );
     readControlPointsFromPP(&destinyLandmarks, _T("D:\\Master Thesis\\PLY Faces\\Reference_picked_points.pp"));
	int controlPointsNum=sourceLandmarks.size();
	matrix<double> mtx_weight( controlPointsNum+4, 4);
	matrix<double> mtx_k(controlPointsNum, controlPointsNum );
     calc_tps_single(sourceLandmarks,destinyLandmarks, &mtx_weight, &mtx_k);
	 LPTSTR srcPointsPath=_T("D:\\Master Thesis\\PLY Faces\\Reference.ply");//input here the path of mesh waiting for TPS translate.
      LPTSTR tgtPointsPath=_T("D:\\Master Thesis\\PLY Faces\\Reference_adjusted.ply");
	 LPTSTR transferedPath=_T("D:\\Master Thesis\\PLY Faces\\M0019A1EnCtrimR0-tps.bjt.txt.ply");//input here the path of mesh been transfered.
      LPTSTR registeredPath= _T("D:\\Master Thesis\\PLY Faces\\M0019A1EnCtrimR0-registered.bjt.txt.ply");	
	 LPTSTR refPath=_T("D:\\Master Thesis\\PLY Faces\\Reference-10000.ply");

	 mrpt::slam::CSimplePointsMap refPoints, srcPoints, alignedPoints, transferedPoints, registeredPoints;
	 readPointsFromPLY(&srcPoints, srcPointsPath );
	 adjustCoord(srcPoints, RZ, transMTX);
	 writePointsToPly(&srcPoints, srcPointsPath,tgtPointsPath);

	 //translate dense points and output results into another PLY file.
	 TPSconverse(srcPoints, alignedPoints, transferedPoints, mtx_weight, sourceLandmarks);
	 writePointsToPly(&transferedPoints, srcPointsPath, transferedPath);

	 readPointsFromPLY(&refPoints, refPath);
	 std::vector<collisionNode> refList;
	 mrpt::slam::CMetricMap::TMatchingPairList pairList;
	 mrpt::poses::CPose3D pose;
	 mrpt::poses::CPoint3D point;
	 float ratio, dist;
	 mrpt::poses::CPose3D init(0,0,0);
	 transferedPoints.computeMatchingWith3D(&refPoints,pose, 500,500, point,pairList, ratio, &dist);
	 for(mrpt::slam::CMetricMap::TMatchingPairList::iterator iter=pairList.begin(); iter!=pairList.end(); iter++)
	 {
		 int refIdx=(*iter).other_idx;
		 int srcIdx=(*iter).this_idx;
		 float x, y, z;
		 srcPoints.getPoint(srcIdx, x, y, z);
		 registeredPoints.insertPoint(x, y, z);
	 }
	 writePointsToPly(&registeredPoints, refPath, registeredPath);
	 */
	

//}

	 /* one2OneCorrespond(refPoints, transferedPoints, refList );
	  for(std::vector<collisionNode>::iterator it=refList.begin(); it!=refList.end(); it++)
	  {
		  float x,y,z;
		  unsigned int idx= (*it).otherIdx;
		  srcPoints.getPoint(idx, x,y,z);
		  registeredPoints.insertPoint(x,y,z);
	  }*/

	   
		/*
		writePointsToPly(&srcPoints,_T("D:\\Master Thesis\\PLY Faces\\M0006A1EnCtrimR0.bjt.txt.ply"), _T("D:\\Master Thesis\\PLY Faces\\M0006A1EnCtrimR0-src.bjt.txt.ply"));
		writePointsToPly(&alignedPoints,_T("D:\\Master Thesis\\PLY Faces\\M0006A1EnCtrimR0.bjt.txt.ply"), _T("D:\\Master Thesis\\PLY Faces\\M0006A1EnCtrimR0-aligned.bjt.txt.ply"));
		writePointsToPly(&transferedPoints,_T("D:\\Master Thesis\\PLY Faces\\M0006A1EnCtrimR0.bjt.txt.ply"), _T("D:\\Master Thesis\\PLY Faces\\M0006A1EnCtrimR0-tps.bjt.txt.ply") );
		*/
	  /*
	  matrix<double> densePoints=readDensePoints(densePointsPath );
	  matrix<double> changedDensePoints=densePoints*mtx_weight;
	  writeDensePoints(changedDensePoints, densePointsChangedName);
	  */
	   /*
  float dist, angularDist, ratio;
	 	 std::vector<collisionNode> *tranferedList=new std::vector<collisionNode>();
	 	 std::vector<collisionNode> refList;
	 	 std::vector<int*> k;
	 	 //establish idx list for tranfered points
	 	 for(long i=0; i<transferedPoints.getPointsCount(); i++)
	 	 { 
	 		 tranferedList->push_back(collisionNode(i));
	 	 }
	 
	 	 //establish correspond List
		 readPointsFromPLY(&refPoints, _T("D:\\Master Thesis\\PLY Faces\\Reference.ply"));
	 	 for(long i=0; i<refPoints.getPointsCount(); i++)
	 	 {
	 		 refList.push_back(collisionNode(i));
	 	 }
	 
	 	long corrPointsNum=0;
	 	BOOL finished=0;//true if all points in reference list are found correspondence.
	 	while(!finished)
	 	{
	 		std::vector<collisionNode*> visitedPoints;
	 		mrpt::slam::CMetricMap::TMatchingPairList pairList;
	 		mrpt::poses::CPose3D pose;
	 		mrpt::poses::CPoint3D point;
	 		float ratio, dist;
	 		CPose3D init(0,0,0);
	 		mrpt::slam::CICP icp;
	 		transferedPoints.computeMatchingWith3D(&refPoints,pose, 500,500, point,pairList, ratio, &dist);
	 		for(mrpt::slam::CMetricMap::TMatchingPairList::iterator iter=pairList.begin(); iter!=pairList.end(); iter++)
	 		{
	 			if(refList.at((*iter).other_idx).visited==0)
	 			{
	 				unsigned int tranferedIdx=(*iter).this_idx;
	 				if(tranferedList->at(tranferedIdx).visited==0)
	 				{
	 					tranferedList->at(tranferedIdx).visited=1;
	 					tranferedList->at(tranferedIdx).otherIdx=(*iter).other_idx;
	 					tranferedList->at(tranferedIdx).minSq=(*iter).errorSquareAfterTransformation;
	 					visitedPoints.push_back(&tranferedList->at(tranferedIdx));
	 				}
	 				if(tranferedList->at(tranferedIdx).visited==1)
	 				{
	 					if(tranferedList->at(tranferedIdx).minSq>(*iter).errorSquareAfterTransformation)
	 					{
	 						tranferedList->at(tranferedIdx).otherIdx=(*iter).other_idx;
	 						tranferedList->at(tranferedIdx).minSq=(*iter).errorSquareAfterTransformation;
	 					}
	 				}
	 			}
	 			
	 
	 		}
*/


/*
	 		for(std::vector<collisionNode>::iterator it=refList.begin(); it!=refList.end(); it++)
	 		{
	 			//if the correspondence for this reference point have not established yet.
	 			//find closest point in transfered points set.
	 			if((*it).otherIdx==-1)
	 			{
	 				float x, y, z;
	 				std::vector<int> idx;
	 				std::vector<float> sq;
	 				referencePoints.getPoint((*it).selfIdx,x,y,z);
	 				transferedPoints.kdTreeNClosestPoint3DIdx(x,y,z,1,idx,sq);
	 				long currentIdx=(*idx.begin());
	 				float dis=(*sq.begin());
	 				if(tranferedList->at(currentIdx).visited==0)
	 				{
	 					tranferedList->at(currentIdx).visited=1;
	 					tranferedList->at(currentIdx).minSq=dis;
	 					tranferedList->at(currentIdx).otherIdx=(*it).selfIdx;
	 					visitedPoints.push_back(&(tranferedList->at(currentIdx)));
	 				}
	 				else if(dis<tranferedList->at(currentIdx).minSq)
	 				{
	 					tranferedList->at(currentIdx).minSq=dis;
	 					tranferedList->at(currentIdx).otherIdx=(*it).selfIdx;
	 				}			
	 			}
	 		}*/

	 
	 
/*
	 		for(std::vector<collisionNode*>::iterator it=visitedPoints.begin(); it!=visitedPoints.end(); it++)
	 		{
	 			//if transfered point already been visited
	 			//add it to the refList, and 
	 			//delete it from the transfered points map and transList
	 				int refIdx=(*it)->otherIdx;
	 				refList.at(refIdx).otherIdx=(*it)->selfIdx;
	 				refList.at(refIdx).minSq=(*it)->minSq;
	 				refList.at(refIdx).visited=1;
	 				corrPointsNum++;
	 		}
	 		std::vector<collisionNode> *temp=tranferedList;
	 		tranferedList=new std::vector<collisionNode>();
	 		std::vector<bool> deleteMask(transferedPoints.getPointsCount());
	 
	 		//delete node from transferedList and transferedPoints
	 		unsigned int k=0;
	 		for(std::vector<collisionNode>::iterator it=temp->begin(); it!=temp->end(); it++)
	 		{
	 
	 			if((*it).visited==0)
	 			{
	 				tranferedList->push_back((*it));
	 				deleteMask.at(k)=0;
	 			}
	 			else deleteMask.at(k)=1;
	 
	 			k++;
	 
	 		}
	 		delete temp;
	 		transferedPoints.applyDeletionMask(deleteMask);
	 		
	 
	 
	 		if (corrPointsNum==refList.size())
	 		{
	 			finished=true;
	 		}
	 	}
	 	mrpt::slam::CSimplePointsMap registeredPoints;
	 	 registerToFile(refList,srcPoints,registeredPoints,_T("D:\\Master Thesis\\PLY Faces\\Reference.ply"));*/

	 	 
	 




/*
matrix<double> readDensePoints(LPTSTR path)
{
	ifstream myfile (path);
	if (myfile.is_open())
	{
		matrix<double>* points=NULL;
		char line[1024]={0};
		long pointsNum=0;
		//get points number first
		while(!myfile.eof())
		{
			myfile.getline(line, 1000);
			string s(line);
			if(s.find("element vertex"))
			{
				pointsNum=atol(s.substr(15,s.length()-15));
				points=new matrix<double>(pointsNum,4);
			}
			else if(s.find("end header")) break;
		}

		if(points!=NULL)
		{
			myfile.getline(line, 1000);
			string s(line);

			//first eat out all empty line
			while(s.size()==0)
			{
				myfile.getline(line,1000);
				s.clear();
				s.append(line);
			}

		//when s is not empty, start to get points
			int i=0;
		while (! myfile.eof() &&i<pointsNum)
		{
				double coord[3];
				getXYZPLY(s,coord);
				(*points)(i,0)=coord[0];
				(*points)(i,1)=coord[1];
				(*points)(i,2)=coord[2];
				(*points)(i,3)=1;
				myfile.getline(line,1000);
				s.clear();
				s.append(line);
				i++;
			}
		}
		}
		myfile.close();
	}
	else 
	{
		cout << "Unable to open file";
		return NULL;
	
	}
}


 writeDensePoints(matrix<double> *DensePoints, LPTSTR densePointsChangedPath)
 {
	 
	 string s("D:\\Master Thesis\\PLY Faces\\");
	 s.append(densePointsChangedPath);
	 s.append(".ply");
	 ofstream saveFile(s.c_str());
	 saveFile<<"ply\n" ;
	 saveFile<<"	format ascii 1.0\n"; 
	 saveFile<<"comment author: Grey Fox\n";
	 saveFile<<"comment object: another cube\n";
	 saveFile<<"element vertex " <<vertexSet.size()<<"\n";
	 saveFile<<"property float x \n";
	 saveFile<<"property float y \n";
	 saveFile<<"property float z \n";
	 saveFile<<"element face "<<polySet.size()<<"\n";
	 saveFile<<"property list uchar int vertex_index \n"  ;
	 saveFile<<"end_header\n";
	 saveFile<<"\n";
	 for(int i=0;i<DensePoints->size1();i++)
	 {
		 saveFile<<(*DensePoints)(i,0)<<" "<<(*DensePoints)(i,1)<<" "<<(*DensePoints)(i,2)<<endl;
	 }
	 saveFile<<"\n";
	 for(vector<Poly*>::iterator i=polySet.begin();i!=polySet.end();i++)
	 {
		 saveFile<<"3 "<<(*i)->v1Num<<" "<<(*i)->v2Num<<" "<<(*i)->v3Num<<"\n";
	 }

 }
*/




