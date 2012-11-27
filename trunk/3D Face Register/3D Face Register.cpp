// 3D Face Register.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "afx.h"
#include "../3DMorphInterface/3DMorphLib.h"

using namespace std;


int _tmain(int argc, _TCHAR* argv[])
{ 
    //SetDllDirectory(
	//   _T("./dlls")
    //);
    LPTSTR refPointsPath=_T(".\\PickedPointsFaces\\reference face\\Reference_adjusted.ply");
    LPTSTR refLandMarksPath=_T(".\\PickedPointsFaces\\reference face\\Reference_adjusted.pp");
    LPTSTR dataDirectoryPath=_T(".\\PickedPointsFaces\\");
    LPTSTR registeredDirectoryPath=_T(".\\PickedPointsFaces\\RegisteredFaces\\");
    LPTSTR srcPointsPath=_T(".\\PLY Faces\\M0019A1EnCtrimR0-adjusted.bjt.txt.ply");
    LPTSTR srcLandMarksPath=_T(".\\PLY Faces\\M0019A1EnCtrimR0-adjusted.bjt.txt_picked_points.pp"); 
    LPTSTR registeredPath=_T(".\\PickedPointsFaces\\RegisteredFaces\\M0019A1EnCtrimR0_adjusted_registered.ply");
    
    morph::modelRegistration(refPointsPath,refLandMarksPath,srcPointsPath,srcLandMarksPath,registeredPath);

/*
    CString allppFilePath;
    allppFilePath.Append(dataDirectoryPath);
    allppFilePath.Append(_T("*.pp"));
    CFileFind ppFinder;
    bool isAny=ppFinder.FindFile(allppFilePath);
    while(isAny!=0)
    {
        isAny=ppFinder.FindNextFile();

        LPTSTR srcPointsPath, srcLandMarksPath, registeredPath;

        CString ppFileName=ppFinder.GetFileTitle();

        CString plyFilePath;
        plyFilePath.Append(dataDirectoryPath);
        plyFilePath.Append(ppFileName);
        plyFilePath.Append(_T(".ply"));
        srcPointsPath=plyFilePath.GetBuffer(0);

        CString registeredFilePath;
        registeredFilePath.Append(registeredDirectoryPath);
        registeredFilePath.Append(ppFileName);
        registeredFilePath.Append(_T("_registered.ply"));
        registeredPath=registeredFilePath.GetBuffer(0);

        CString ppFilePath=ppFinder.GetFilePath();
        srcLandMarksPath=ppFilePath.GetBuffer(0);

        morph::modelRegistration(refPointsPath,refLandMarksPath,srcPointsPath,srcLandMarksPath,registeredPath);
    }*/

    
    return 0;
}

