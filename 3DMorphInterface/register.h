#ifndef REGISTER_H
#define REGISTER_H
#include <vector>
#include <Windows.h>

#include "../external/include/mrpt/slam/CSimplePointsMap.h"
#include "../external/include/mrpt/base.h"

class collisionNode  
{
public:
	unsigned int otherIdx;
	unsigned int selfIdx;
	float minSq;
	bool visited;
	bool corresponded;
	collisionNode(long selfIdx)
	{
		this->selfIdx=selfIdx;
		this->otherIdx=-1;
		this->minSq=99999999;
		this->visited=0;
		this->corresponded=0;
	}
};

//function declaration.
void registerToFile(std::vector<collisionNode> &corrIdx, 
	mrpt::slam::CSimplePointsMap &sourcePoints,
	mrpt::slam::CSimplePointsMap &registeredPoints, 
	LPTSTR referencePath);

void one2OneCorrespond(mrpt::slam::CSimplePointsMap refPoints, 
	mrpt::slam::CSimplePointsMap transferedPoints, 
	std::vector<collisionNode> &refList);

#endif