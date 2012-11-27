#include "stdafx.h"

#include "../external/include/mrpt/utils/TMatchingPair.h"
#include "../external/include/mrpt/slam/CICP.h"

#include "register.h"
#include "FileIO.h"

using namespace std;
void registerToFile(std::vector<collisionNode> &corrIdx, mrpt::slam::CSimplePointsMap &sourcePoints,mrpt::slam::CSimplePointsMap &registeredPoints, LPTSTR referencePath)
{
	ifstream inputFile(referencePath);
	ofstream outputFile(_T("D:\\Master Thesis\\PLY Faces\\registered.ply"));
	if(inputFile.is_open())
	{
		int status=READING_HEADER;
		int pointsNum=0;
		int count=0;
		std::vector<collisionNode>::iterator it=corrIdx.begin();
		while(!inputFile.eof())
		{
			char line[1000];
			inputFile.getline(line,1000);
			string s(line);

			switch(status)
			{

				//just record header.
			case READING_HEADER:
				{
					outputFile<<s<<endl;
					//record vertex number.
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
			case READING_VERTEX:
				{
					//eat up all empty lines
					if(s.size()==0&&count==0)
					{					
					}
					else if(count<pointsNum)
					{
						float x, y, z;
						sourcePoints.getPoint((*it).otherIdx, x, y, z);
						registeredPoints.insertPoint(x, y, z);
						outputFile<<x<<" "<<y<<" "<<z<<endl;
						count++;
						it++;
					}
					else if(count==pointsNum)
					{
						outputFile<<s<<endl;
						status=READING_POLY;
					}
					break;					 
				}
				//copy poly data
			case READING_POLY:
				{
					outputFile<<s<<endl;
					break;
				}
			}			
		}
	}
}

void one2OneCorrespond(mrpt::slam::CSimplePointsMap refPoints, mrpt::slam::CSimplePointsMap transferedPoints, vector<collisionNode> &refList)
{
	float dist, angularDist, ratio;
	std::vector<collisionNode> *tranferedIdxList=new std::vector<collisionNode>();
	std::vector<collisionNode> *refIdxList=new std::vector<collisionNode>();
	std::vector<int*> k;
	//establish idx list for transfered points in order to
	//track the true idx of deleted points in 
	//transferedPoints and refPoints
	for(long i=0; i<transferedPoints.getPointsCount(); i++)
	{ 
		tranferedIdxList->push_back(collisionNode(i));
	}

	for(long i=0; i<refPoints.getPointsCount(); i++)
	{
		refIdxList->push_back(collisionNode(i));
		refList.push_back(collisionNode(i));
	}

	long corrPointsNum=0;
	BOOL finished=0;//true if all points in reference list are found correspondence.
	while(!finished)
	{
		std::vector<collisionNode*> visitedPoints;
		mrpt::utils::TMatchingPairList pairList;
		mrpt::poses::CPose3D pose;
		mrpt::poses::CPoint3D point;
		float ratio, dist;
		mrpt::poses::CPose3D init(0,0,0);
		mrpt::slam::CICP icp;
		transferedPoints.computeMatchingWith3D(&refPoints,pose, 500,500, point,pairList, ratio, &dist);
		for(mrpt::utils::TMatchingPairList::iterator iter=pairList.begin(); iter!=pairList.end(); iter++)
		{
			unsigned int fakeTranferedIdx=(*iter).this_idx;
			unsigned int trueTranferedIdx=tranferedIdxList->at(fakeTranferedIdx).selfIdx;
			unsigned int fakeRefIdx=(*iter).other_idx;
			unsigned int trueRefIdx=refIdxList->at(fakeRefIdx).selfIdx;
			if(tranferedIdxList->at(fakeTranferedIdx).visited==0)
			{
				tranferedIdxList->at(fakeTranferedIdx).visited=1;
				tranferedIdxList->at(fakeTranferedIdx).otherIdx=trueRefIdx;
				tranferedIdxList->at(fakeTranferedIdx).minSq=(*iter).errorSquareAfterTransformation;
				visitedPoints.push_back(&tranferedIdxList->at(fakeTranferedIdx));
			}
			if(tranferedIdxList->at(fakeTranferedIdx).visited==1)
			{
				if(tranferedIdxList->at(fakeTranferedIdx).minSq>(*iter).errorSquareAfterTransformation)
				{
					tranferedIdxList->at(fakeTranferedIdx).otherIdx=trueRefIdx;
					tranferedIdxList->at(fakeTranferedIdx).minSq=(*iter).errorSquareAfterTransformation;
				}
			}
		}
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
		//delete node from transferedList 
		std::vector<collisionNode> *temp=tranferedIdxList;
		tranferedIdxList=new std::vector<collisionNode>();
		std::vector<bool> transferedDeleteMask(transferedPoints.getPointsCount());
		unsigned int k=0;
		for(std::vector<collisionNode>::iterator it=temp->begin(); it!=temp->end(); it++)
		{
			if((*it).visited==0)
			{
				tranferedIdxList->push_back((*it));
				transferedDeleteMask.at(k)=0;
			}
			else transferedDeleteMask.at(k)=1;
			k++;
		}
		delete temp;
		transferedPoints.applyDeletionMask(transferedDeleteMask);

		//delete corresponded ref Points.
		std::vector<collisionNode> *temp2=refIdxList;
		refIdxList=new std::vector<collisionNode>();
		std::vector<bool> refDeletionMask(refPoints.getPointsCount());
		k=0;
		for(std::vector<collisionNode>::iterator it=temp2->begin(); it!=temp2->end(); it++)
		{
			unsigned int trueRefIdx=(*it).selfIdx;
			if(refList.at(trueRefIdx).visited==0)
			{
				refIdxList->push_back(refList.at(trueRefIdx));
				refDeletionMask.at(k)=false;
			}
			else
			{
				refDeletionMask.at(k)=true;
			}
			k++;
		}
		delete temp2;
		refPoints.applyDeletionMask(refDeletionMask);
		if (corrPointsNum==refList.size())
		{
			finished=true;
		}
	}
}

void closestPointMatching(mrpt::slam::CSimplePointsMap refPoints, mrpt::slam::CSimplePointsMap transferedPoints, vector<collisionNode> &refList)
{

}