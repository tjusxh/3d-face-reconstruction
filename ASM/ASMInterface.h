#ifndef ASMINTERFACE_H
#define ASMINTERFACE_H
#include <iostream>
#include <vector>
namespace ASM
{
typedef struct{

public:
	double x, y;
	int idx;
}ASMPoint;

extern "C"
{
 _declspec(dllexport) void initASM();
 _declspec(dllexport) void getASMPoints(const char *imgName, std::vector<ASMPoint> * pointsSet, int &width, int  &height);
};

}
#endif 
