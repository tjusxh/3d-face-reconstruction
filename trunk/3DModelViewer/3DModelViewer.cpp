// 3DModelViewer.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <windows.h>
#include <string.h>
#include <limits>
#include <iostream>
#include <Commdlg.h>

#include "../external/include/GL/glui.h"
#include "../external/src/stasm3.1/stasm/ASMInterface.h"
#include "../external/include/opencv/cv.h"
#include "../external/include/opencv/cxcore.h"
#include "../external/include/opencv/highgui.h"

#include "../nonNegLeastSqFit/nonNegLeastSqFit.h"
#include "../GLMlib/glm.h"
#include "../3DMorphInterface/3DMorphLib.h"
#include "../nonNegLeastSqFit/nonNegLeastSqFit.h"


#define LOAD_IMG 300
#define  GEN_DISLANDMARKS 400
#define  TPS_TRANSFORM 401
#define  TEXTURE_MAPPING 402
#define  SELECT_MODEL 403
#define  COMPULSORY_ADJUST 404
#define MODEL_RESET 405

using namespace morph;
using namespace std;


const char* PATH_REFERENCE_FACE = ".\\PickedPointsFaces\\reference face\\Reference_adjusted.obj";

typedef struct _cell {
	int id;
	int x, y;
	float min, max;
	float value;
	float step;
	char* info;
	char* format;
} cell;

cell lookat[9] = {
	{ 1, 180, 120, -5.0, 5.0, 0.0, 0.1,
	"Specifies the X position of the eye point.", "%.2f" },
	{ 2, 240, 120, -5.0, 5.0, 3.0, 0.1,
	"Specifies the Y position of the eye point.", "%.2f" },
	{ 3, 300, 120, -5.0, 5.0, 0.0, 0.1,
	"Specifies the Z position of the eye point.", "%.2f" },
	{ 4, 180, 160, -5.0, 5.0, 0.0, 0.1,
	"Specifies the X position of the reference point.", "%.2f" },
	{ 5, 240, 160, -5.0, 5.0, 0.0, 0.1,
	"Specifies the Y position of the reference point.", "%.2f" },
	{ 6, 300, 160, -5.0, 5.0, 0.0, 0.1,
	"Specifies the Z position of the reference point.", "%.2f" },
	{ 7, 180, 200, -2.0, 2.0, 0.0, 0.1,
	"Specifies the X direction of the up vector.", "%.2f" },
	{ 8, 240, 200, -2.0, 2.0, 0.0, 0.1,
	"Specifies the Y direction of the up vector.", "%.2f" },
	{ 9, 300, 200, -2.0, 2.0, 1.0, 0.1,
	"Specifies the Z direction of the up vector.", "%.2f" },
};

cell perspective[4] = {
	{ 10, 180, 80, 1.0, 179.0, 10, 1.0,
	"Specifies field of view angle (in degrees) in y direction.", "%.1f" },
	{ 11, 240, 80, -3.0, 3.0, 1.0, 0.01,
	"Specifies field of view in x direction (width/height).", "%.2f" },
	{ 12, 300, 80, 0.1, 10.0, 0.5, 0.05,
	"Specifies distance from viewer to near clipping plane.", "%.1f" },
	{ 13, 360, 80, 0.1, 10.0, 30, 0.05,
	"Specifies distance from viewer to far clipping plane.", "%.1f" },
};

cell frustum[6] = {
	{ 14, 120, 80, -10.0, 10.0, -1.0, 0.1,
	"Specifies coordinate for left vertical clipping plane.", "%.2f" },
	{ 15, 180, 80, -10.0, 10.0, 1.0, 0.1,
	"Specifies coordinate for right vertical clipping plane.", "%.2f" },
	{ 16, 240, 80, -10.0, 10.0, -1.0, 0.1,
	"Specifies coordinate for bottom vertical clipping plane.", "%.2f" },
	{ 17, 300, 80, -10.0, 10.0, 1.0, 0.1,
	"Specifies coordinate for top vertical clipping plane.", "%.2f" },
	{ 18, 360, 80, 0.1, 5.0, 1.0, 0.01,
	"Specifies distance to near clipping plane.", "%.2f" },
	{ 19, 420, 80, 0.1, 5.0, 3.5, 0.01,
	"Specifies distance to far clipping plane.", "%.2f" },
};

cell ortho[6] = 
{
	{ 14, 120, 80, -10.0, 10.0, -1.0, 0.1,
	"Specifies coordinate for left vertical clipping plane.", "%.2f" },
	{ 15, 180, 80, -10.0, 10.0, 1.0, 0.1,
	"Specifies coordinate for right vertical clipping plane.", "%.2f" },
	{ 16, 240, 80, -10.0, 10.0, -1.0, 0.1,
	"Specifies coordinate for bottom vertical clipping plane.", "%.2f" },
	{ 17, 300, 80, -10.0, 10.0, 1.0, 0.1,
	"Specifies coordinate for top vertical clipping plane.", "%.2f" },
	{ 18, 360, 80, -5.0, 5.0, 1.0, 0.01,
	"Specifies distance to near clipping plane.", "%.2f" },
	{ 19, 420, 80, -5.0, 5.0, 30, 0.01,
	"Specifies distance to far clipping plane.", "%.2f" },
};

enum 
{
	PERSPECTIVE, FRUSTUM, ORTHO
} mode = PERSPECTIVE;


/* Indicators */
bool isFeaturePointsFounded=false; //indicate whether feature points are founded;
bool isPVTexGenerated=false; //indicated whether texture for Photo Viewer has been generated
bool isModelTexGenerated=false; //indicate whether texture for model has been generated
bool isVertexCoordGenerated=false; //indicate whether the polygon vertex has been determined

int isDrawSrcMarks=false, isDrawDisMarks=false;//indicate whether need to draw landmarks
int isDrawModel=true; 
int isDrawFeaturePoints=true;//indicate whether draw feature points
int isLinkFeaturePoints=false;//indicate whether link feature points
bool isModelUpdated=false;//indicate whether the model is updated 
bool isASMSetted=false;//indicate wether ASM model is established

bool isDislandmarksGenerated=false;//indicate whether dislandmarks is generated
OPENFILENAMEA ofn ;
char szFile[100];

/* Photo Viewer Variables */
typedef struct  
{
	std::vector<ASM::ASMPoint> pointsSet; // vector store feature points
	double max_x, max_y, min_x, min_y;  // feature points bound
	int pivotIdx; // the idx of the feature points on the nose
	int width, height; //img width and height
	void clear()
	{
		pointsSet.clear();
		pivotIdx=-1;
		max_x=(numeric_limits<double>::min)();
		max_y=(numeric_limits<double>::min)();
		min_x=(numeric_limits<double>::max)();
		min_y=(numeric_limits<double>::max)(); 

	}
}ASMPointSet;

ASMPointSet featurePoints; // store feature points

IplImage *rawPhoto=NULL;// store picture for texture mapping
BYTE *dataForPV,*dataForModel;//texture data

GLuint textureForPV=0;//store PV texture ID num
GLuint textureForScr=0;//store screen texture ID num

double vertexCoordForPV[8], texCoordForPV[2]; //four corresponding coordinates for vertex and texture imgs
double faceRegionWidth, faceRegionHeight, polygonWidth, polygonHeight; //size of polygon and texture imgs
int PVwidth, PVheight; // store photoViewer window size
int PVmin;
double redWnd_x, redWnd_y; //half of the length that PV wnd may exceed the polygon along X or Y coordinate
double disThreshold=5; //set the max hit dis that considered selected 
int pickedIdx=-1;//set the picked up point's idx

/* Screen Variables */
GLMmodel* model_unitized = NULL;//store unitized reference model
double photoToModel=0;//store photo to model scale
std::vector<std::vector<morphPoint>*> densePointsSet;
std::vector<std::vector<morphPoint>*>  landmarkSet;
std::vector<morphPoint> srcModelLandmark, disModelLandmark;
std::vector<morphPoint> originMod;// record unTransformed dense points in original coord and scale
std::vector<morphPoint> transformedMod; // record transformed dense points in original coord and scale
std::vector<double> fitSolution;
std::vector<double> fittedPoints;
double fitScale;

/* Other Variables */
GLboolean world_draw = GL_TRUE;

GLfloat *adjustPara;
GLint selection = 0;

GLdouble projection[16], modelview[16], inverse[16];
GLuint window, world, screen, command, photoViewer, fileSelectorWnd;
GLuint sub_width = 256, sub_height = 256;

GLvoid *font_style = GLUT_BITMAP_TIMES_ROMAN_10;

GLUI *glui, *glui2, *fileBrowserGlui; //the control board
GLUI_Panel      *obj_panel, *obj_panel2;
void redisplay_all(void);
void myGlutIdle( void )
{
	/* According to the GLUT specification, the current window is 
	undefined during an idle callback.  So we need to explicitly change
	it if necessary */
	if ( glutGetWindow() != window ) 
		glutSetWindow(window);  

	/*  GLUI_Master.sync_live_all();  -- not needed - nothing to sync in this
	application  */

	glutPostRedisplay();
}




/************************************************************************/
/* Other functions                                                                     */
/************************************************************************/
void linearLeastSquare(
					   const ASMPointSet &disPoints, 
					   const std::vector<std::vector<morphPoint>*>  &factorVectors, 
					   std::vector<double> &outputPara, 
					   std::vector<double> &outputCombine,
					   double &outputScale
					   )
{
	int landMarksNum2D=disPoints.pointsSet.size();
	int landMarksNum3D=factorVectors.at(0)->size();
	int rowsNum=landMarksNum2D*2;
	int columnNum=factorVectors.size();
	CvMat *factorMtx=cvCreateMat(rowsNum, columnNum, CV_64FC1);
	CvMat *factorMtx3D=cvCreateMat(landMarksNum3D*3, columnNum, CV_64FC1);
	CvMat *c=cvCreateMat(columnNum, 1, CV_64FC1);
	CvMat *chinPoints=cvCreateMat(1, columnNum, CV_64FC1 ); //just record the z coord of all chin points in factorvectors.
	CvMat *disMtx=cvCreateMat(rowsNum, 1, CV_64FC1);

	

	int chinPointPos=7;
	double chinYInDis=disPoints.pointsSet.at(chinPointPos).y;
	int nosePointPos=63;

	//initiate factorMtx
	for(int i=0; i<factorMtx->rows; i++)
	{
		double *ptr=(double *)(factorMtx->data.ptr+factorMtx->step*i);
		for(int j=0; j<factorMtx->cols; j++)
		{
			int mod=i%2;
			int np=i/2;
			if(mod==0)
			{
				*ptr=factorVectors.at(j)->at(np).x;
			}
			else if(mod==1)
			{
				*ptr=factorVectors.at(j)->at(np).z;
			}
			ptr++;
		}
	}

	for(int i=0; i<factorMtx3D->rows; i++)
	{
		double *ptr=(double *)(factorMtx3D->data.ptr+factorMtx3D->step*i);
		for(int j=0; j<factorMtx3D->cols; j++)
		{
			int mod=i%3;
			int np=i/3;
			if(mod==0)
			{
				*ptr=factorVectors.at(j)->at(np).x;
			}
			else if(mod==1)
			{
				*ptr=factorVectors.at(j)->at(np).y;
			}
			else if(mod==2)
			{
				*ptr=factorVectors.at(j)->at(np).z;
			}
			ptr++;
		}
	}

	//initiate disMtx
	for(int i=0; i<disMtx->rows; i++)
	{
		double *ptr=(double *)(disMtx->data.ptr+disMtx->step*i);
		for(int j=0; j<disMtx->cols; j++)
		{
			int mod=i%2;
			int np=i/2;
			if(mod==0)
			{
			  *ptr=disPoints.pointsSet.at(np).x-disPoints.pointsSet.at(63).x;
			}
			if(mode==1)
			{
				*ptr=disPoints.pointsSet.at(np).y-disPoints.pointsSet.at(63).y;
			}
			ptr++;
		}
	}

	cvSolve(factorMtx, disMtx, c, CV_SVD_SYM);
	for(int i=0; i<c->rows; i++)
	{
		double *ptr=(double *)(c->data.ptr+c->step*i);
		for(int j=0; j<c->cols; j++)
		{
			outputPara.push_back(*ptr);
			ptr++;
		}
	}

	//output combine result
	CvMat *combineRes=cvCreateMat(landMarksNum3D*3,1, CV_64FC1);
	cvGEMM(factorMtx3D,c,1,NULL,0,combineRes);
	for(int i=0; i<combineRes->rows; i++)
	{
		double *ptr=(double *)(combineRes->data.ptr+combineRes->step*i);
		for(int j=0; j<combineRes->cols; j++)
		{
			outputCombine.push_back(*ptr);
			ptr++;
		}
	}

	//output scale factor;
	double total=0;
	for(int i=0; i<outputPara.size(); i++)
	{
		total+=outputPara.at(i);
	}
	outputScale=1/total;
     
	// Release Matrix
	cvReleaseMat(&factorMtx);
	cvReleaseMat(&c);
	cvReleaseMat(&chinPoints);
	cvReleaseMat(&disMtx);
	cvReleaseMat(&combineRes);

}

void genDisLandmarks()
{
	/* Non-negative least square calculationg */
	int m=2*featurePoints.pointsSet.size();
	int n=landmarkSet.size();
	int mda=m;

	double *a = new double[m*n];
	for(int i=0; i<n; i++)
	{
		for(int j=0; j<m; j++)
		{
			int num=j/2;
			int mod=j%2;
			if(mod==0)
			{
				a[i*m+j]=landmarkSet.at(i)->at(num).x;
			}
			else
			{
				a[i*m+j]=landmarkSet.at(i)->at(num).z;
			}
		}
	}


	double *b=new double[m];
	for(int i=0 ;i <m; i++)
	{
		int num=i/2;
		int mod=i%2;
		if(mod==0)
		{
			b[i]=featurePoints.pointsSet.at(63).x-featurePoints.pointsSet.at(num).x;
		}
		else
		{
			b[i]=featurePoints.pointsSet.at(num).y-featurePoints.pointsSet.at(63).y;
		}
	}
	std::cout<<endl;

	std::cout<<"b:"<<endl;
	for(int i=0; i<m; i++)
	{
		std::cout<<b[i]<<",";
	}
	std::cout<<endl<<endl;
/*
     //just for test
	for(int i=0; i<m; i++)
	{
		int num=i/2;
		int mod=i%2;
		if(mod==0)
		{
			b[i]=landmarkSet.at(1)->at(num).x;
		}
		else
		{
			b[i]=landmarkSet.at(1)->at(num).z;
		}
	}
	//*/


	std::cout<<"b:"<<endl;
	for(int i=0; i<m; i++)
	{
		std::cout<<b[i]<<",";
	}
	std::cout<<endl<<endl;

	double *x=new double[n];
	double rnorm;
	int  mode;
	int *index=new int[m];
	double* w=new double[n];
	double *z=new double[m];
	std::cout<<"a:"<<endl;
	for(int i=0; i<n; i++)
	{
		for(int j=0; j<m ; j++)
		{
			std::cout<<a[i*m +j]<<",";
		}
		std::cout<<endl;
	}
	std::cout<<endl;




	nnls_(a,&mda,&m,&n,b, x, &rnorm, w, z, index,&mode);

	std::cout<<endl;
	std::cout<<"x= ";
	for(int i=0; i<n; i++)
	{
		std::cout<<x[i]<<",";
	}
	std::cout<<endl;

	
	for(int i=0; i<n ; i++)
	{
		photoToModel+=x[i];
	}
	//build dislandmarks
	disModelLandmark.clear();
	for(int i=0; i<srcModelLandmark.size(); i++)
	{
		morphPoint p;
		p.x=0; p.y=0; p.z=0;
		disModelLandmark.push_back(p);
	}
	//weighted add all landmarks
	for(int i=0; i<n; i++)
	{
		double weight=x[i]/photoToModel;
		for(int j=0; j<srcModelLandmark.size(); j++)
		{
			disModelLandmark.at(j).x+=weight*landmarkSet.at(i)->at(j).x;
			disModelLandmark.at(j).y+=weight*landmarkSet.at(i)->at(j).y;
			disModelLandmark.at(j).z+=weight*landmarkSet.at(i)->at(j).z;
		}
	}
	for(int i=0; i<srcModelLandmark.size();i++)
	{
		srcModelLandmark.at(i).x=disModelLandmark.at(i).x;
		srcModelLandmark.at(i).y=disModelLandmark.at(i).y;
		srcModelLandmark.at(i).z=disModelLandmark.at(i).z;
	}
	//reset originMod
	for(int j=0; j<originMod.size(); j++)
	{
		originMod.at(j).x=0;
		originMod.at(j).y=0;
		originMod.at(j).z=0;
	}
	//weighted add dense points
	for(int i=0; i<n; i++)
	{
		double weight=x[i]/photoToModel;
		for(int j=1; j<originMod.size(); j++)
		{
			originMod.at(j).x+=weight*densePointsSet.at(i)->at(j-1).x;
			originMod.at(j).y+=weight*densePointsSet.at(i)->at(j-1).y;
			originMod.at(j).z+=weight*densePointsSet.at(i)->at(j-1).z;
		}
	}
	//update current model
	for(int j=0; j<originMod.size(); j++)
	{
		model_unitized->vertices[3*j]=originMod.at(j).x;
		model_unitized->vertices[3*j+1]=originMod.at(j).y;
		model_unitized->vertices[3*j+2]=originMod.at(j).z;

	}
	isModelUpdated=true;

	delete[] a;
	delete[] b;
	delete[] x;
	delete[] index;
	delete[] w;
	delete[] z;


	isDislandmarksGenerated=true;
	isDrawDisMarks=true;
}

void forceAdjustDisLM()
{
	if(isDislandmarksGenerated==true)
	{
		//Compulsorily move all points
/*
		double lengthInPhoto=abs(featurePoints.pointsSet.at(63).y-featurePoints.pointsSet.at(7).y);
		double lenghtInModel=abs(srcModelLandmark.at(63).z-srcModelLandmark.at(7).z);
		double scale=lenghtInModel/lengthInPhoto;
*/

		double max_x=0;
		double max_z=0;
		int numLM=disModelLandmark.size();
		int numFP=featurePoints.pointsSet.size();
		for(int i=0 ; i<numLM ; i++)
		{
				disModelLandmark.at(i).x=(featurePoints.pointsSet.at(63).x-featurePoints.pointsSet.at(i).x)/photoToModel;
				disModelLandmark.at(i).z=(featurePoints.pointsSet.at(i).y-featurePoints.pointsSet.at(63).y)/photoToModel;
		
			if(max_x<disModelLandmark.at(i).x) max_x=disModelLandmark.at(i).x;
			if(max_z<disModelLandmark.at(i).z) max_z=disModelLandmark.at(i).z;
		}
		isDrawDisMarks=true;
	}
}
	/*
	//initiate chinPoints
		for(int i=0; i<chinPoints->rows; i++)
		{
			double *ptr=(double*)chinPoints->data.ptr+chinPoints->step*i;
			for(int j=0; j<chinPoints->cols; j++)
			{
				*ptr=factorVectors.at(j)->at(7).z;
				ptr++;
			}
		}*/
	

/*
	CvMat *PM=cvCreateMat(rowsNum, columnNum, CV_64FC1);
	cvGEMM(disMtx,chinPoints,1/chinYInDis, NULL,0, PM);
	CvMat *finalFactor=cvCreateMat(rowsNum, columnNum, CV_64FC1);
	cvSub(factorMtx,PM,finalFactor);*/


	//solve and output c
	/*
CvMat *zeroCol=cvCreateMat(rowsNum,1,CV_64FC1);
	//set to 0
	for(int i=0; i<zeroCol->rows; i++)
	{
		double *ptr=(double *)zeroCol->data.ptr+zeroCol->step*i;
		for(int j=0; j<zeroCol->cols; j++)
		{
			*ptr=0;
			ptr++;
		}
	}
	cvSolve(finalFactor,zeroCol,c,CV_SVD);
*/





/*
	CvMat *scaleMat=cvCreateMat(1,1, CV_64FC1);
	cvGEMM(chinPoints, c, 1/chinYInDis,NULL,0,scaleMat);
	outputScale=*scaleMat->data.ptr;*/





/*
	cvReleaseMat(&PM);
	cvReleaseMat(&finalFactor);
	cvReleaseMat(&zeroCol);*/


	/*cvReleaseMat(&scaleMat);*/



void unitizeAdjust(GLfloat* vertices, int verticeNum)
{
	//transform first, then scale
	for(int i=0; i<verticeNum; i++)
	{
		vertices[3*i]-=adjustPara[0];
		vertices[3*i+1]-=adjustPara[1];
		vertices[3*i+2]-=adjustPara[2];
		vertices[3*i]*=adjustPara[3];
		vertices[3*i+1]*=adjustPara[3];
		vertices[3*i+2]*=adjustPara[3];
	}
}

void drawLandMarks(const std::vector<morphPoint> &landMarks)
{
	int verticeNum=landMarks.size();
	GLfloat * vertices=new GLfloat[verticeNum*3];
	for(int i=0; i<verticeNum; i++)
	{
		vertices[3*i]=landMarks.at(i).x;
		vertices[3*i+1]=landMarks.at(i).y;
		vertices[3*i+2]=landMarks.at(i).z;
	}
	unitizeAdjust(vertices, verticeNum);
	glBegin(GL_POINTS);
	for(int i=0; i<verticeNum; i++)
	{
		glVertex3f(vertices[3*i], vertices[3*i+1], vertices[3*i+2]);
	}
	glEnd();
	delete[] vertices;
}


void
setfont(char* name, int size)
{
	font_style = GLUT_BITMAP_HELVETICA_10;
	if (strcmp(name, "helvetica") == 0) {
		if (size == 12) 
			font_style = GLUT_BITMAP_HELVETICA_12;
		else if (size == 18)
			font_style = GLUT_BITMAP_HELVETICA_18;
	} else if (strcmp(name, "times roman") == 0) {
		font_style = GLUT_BITMAP_TIMES_ROMAN_10;
		if (size == 24)
			font_style = GLUT_BITMAP_TIMES_ROMAN_24;
	} else if (strcmp(name, "8x13") == 0) {
		font_style = GLUT_BITMAP_8_BY_13;
	} else if (strcmp(name, "9x15") == 0) {
		font_style = GLUT_BITMAP_9_BY_15;
	}
}

void 
drawstr(GLuint x, GLuint y, char* format, ...)
{
	va_list args;
	char buffer[255], *s;

	va_start(args, format);
	vsprintf(buffer, format, args);
	va_end(args);

	glRasterPos2i(x, y);
	for (s = buffer; *s; s++)
		glutBitmapCharacter(font_style, *s);
}

void  TPStransModel()
{
	if(isDislandmarksGenerated)
	{
		transformedMod.clear();
		morph::TPStransform(srcModelLandmark, disModelLandmark, &originMod, &transformedMod);

		//update the transformed coord to the unitized model 
		for(unsigned int i=0; i<=model_unitized->numvertices;i++)
		{
			model_unitized->vertices[3*i]=transformedMod.at(i).x;
			model_unitized->vertices[3*i+1]=transformedMod.at(i).y;
			model_unitized->vertices[3*i+2]=transformedMod.at(i).z;
		}
		isModelUpdated=true; // Set the modelupdated to true, update its normals later.
	}
}

void TPStransModelFromFile(LPTSTR disLandmarksPath)
{
	//read dislandmarks
	disModelLandmark.clear();
	morph::readControlPointsFromPP(disModelLandmark ,disLandmarksPath);
     TPStransModel();
}


/*  
*   generate texture coord for model, and cut out right sized textures
*   according to given model and landmarks. This method should only 
*   be called when the 3D and 2D feature points are fully mapped.
*/
void genTexForModel(const std::vector<morphPoint> &modelVertices, const std::vector<morphPoint> &landMarks, const ASMPointSet &featrueP2D)
{
	//get the bound coord of the transformed the model
	double mdminx, mdmaxx, mdminy, mdmaxy, mdminz, mdmaxz;
	mdminx=mdmaxx=modelVertices.at(6).x;
	mdminy=mdmaxy=modelVertices.at(6).y;
	mdminz=mdmaxz=modelVertices.at(6).z;
     //Notice: skip the first one !!!
	for(int i=1; i<modelVertices.size(); i++)
	{
		if (mdmaxx < modelVertices.at(i).x)
			mdmaxx = modelVertices.at(i).x;
		if (mdminx > modelVertices.at(i).x)
			mdminx = modelVertices.at(i).x;

		if (mdmaxy < modelVertices.at(i).y)
			mdmaxy =  modelVertices.at(i).y;
		if (mdminy >  modelVertices.at(i).y)
			mdminy =  modelVertices.at(i).y;

		if (mdmaxz <  modelVertices.at(i).z)
			mdmaxz = modelVertices.at(i).z;
		if (mdminz > modelVertices.at(i).z)
			mdminz = modelVertices.at(i).z;
	}


	/*get Bound of landmarks */
	double lmminx, lmmaxx, lmminy, lmmaxy, lmminz, lmmaxz;
	lmminx=lmmaxx=landMarks.at(6).x;
	lmminy=lmmaxy=landMarks.at(6).y;
	lmminz=lmmaxz=landMarks.at(6).z;
	for(int i=0; i<landMarks.size(); i++)
	{
		if (lmmaxx < landMarks.at(i).x)
			lmmaxx = landMarks.at(i).x;
		if (lmminx > landMarks.at(i).x)
			lmminx = landMarks.at(i).x;

		if (lmmaxy < landMarks.at(i).y)
			lmmaxy = landMarks.at(i).y;
		if (mdminy > landMarks.at(i).y)
			lmminy = landMarks.at(i).y;

		if (lmmaxz < landMarks.at(i).z)
			lmmaxz = landMarks.at(i).z;
		if (lmminz > landMarks.at(i).z)
			lmminz = landMarks.at(i).z;
	}

	//set the pivot to the 63th points (the nose point)
	double pivotxlm=landMarks.at(63).x;
	double pivotylm=landMarks.at(63).y;
	double pivotzlm=landMarks.at(63).z;

	/* get bound of 2D feature points */ 
	double minx2d, maxx2d, miny2d, maxy2d; 
	minx2d=maxx2d=featrueP2D.pointsSet.at(6).x;
     miny2d=maxy2d=featrueP2D.pointsSet.at(6).y;
	 for(int i=0; i<featurePoints.pointsSet.size(); i++)
	 {
		 if (maxx2d < featrueP2D.pointsSet.at(i).x)
			 maxx2d = featrueP2D.pointsSet.at(i).x;
		 if (minx2d> featrueP2D.pointsSet.at(i).x)
			minx2d = featrueP2D.pointsSet.at(i).x;

		 if (maxy2d < featrueP2D.pointsSet.at(i).y)
			 maxy2d = featrueP2D.pointsSet.at(i).y;
		 if (miny2d > featrueP2D.pointsSet.at(i).y)
			 miny2d = featrueP2D.pointsSet.at(i).y;
	 }
      //set the pivot to the 63th points (the nose point)
	 double pivotx2D=featrueP2D.pointsSet.at(63).x;
	 double pivoty2D=featrueP2D.pointsSet.at(63).y;

	 double dis3D=abs(landMarks.at(7).z-landMarks.at(63).z);
	 double dis2D=abs(featrueP2D.pointsSet.at(7).y- featrueP2D.pointsSet.at(63).y);

	 double scale3Dto2D=dis2D/dis3D;

     //find the max dis to pivot point in coodinate x and z on model
	 double maxDis3D=max(max(abs(pivotzlm-mdminz),abs(pivotzlm-mdmaxz)),max(abs(pivotxlm-mdminx),abs(pivotxlm-mdmaxx)));
     
	 //set origin to the left down most
	 double originx3D=pivotxlm-maxDis3D;
	 double originz3D=pivotzlm-maxDis3D;

	 //generate tex coord for model
	 if (model_unitized->texcoords)
		 free(model_unitized->texcoords);
	 model_unitized->numtexcoords = model_unitized->numvertices;
	 model_unitized->texcoords=(GLfloat*)malloc(sizeof(GLfloat)*2*(model_unitized->numtexcoords+1));
	 for(int i=1; i<modelVertices.size(); i++)
	 {
		 double x = modelVertices.at(i).x -originx3D;
		 double y = modelVertices.at(i).z -originz3D;
		 model_unitized->texcoords[2*i]=x/(2*maxDis3D);
		 model_unitized->texcoords[2*i+1]=y/(2*maxDis3D);
	 }

	 /* go through and put texture coordinate indices in all the triangles */
	 GLMgroup *group = model_unitized->groups;
	 while(group) 
	 {
		 for(int i = 0; i < group->numtriangles; i++) 
		 {
			 model_unitized->triangles[(group->triangles[i])].tindices[0] = model_unitized->triangles[(group->triangles[i])].vindices[0];
			 model_unitized->triangles[(group->triangles[i])].tindices[1] = model_unitized->triangles[(group->triangles[i])].vindices[1];
			 model_unitized->triangles[(group->triangles[i])].tindices[2] = model_unitized->triangles[(group->triangles[i])].vindices[2];
		 }    
		 group = group->next;
	 }

	 /* cut the image into a right shape and generate texture */

	 double originx2D=pivotx2D-maxDis3D*scale3Dto2D;
	 double originy2D=pivoty2D-maxDis3D*scale3Dto2D;
	 double maxDis2D=maxDis3D*scale3Dto2D;  
	 //cut the region face for texture
	 cvSetImageROI(rawPhoto, cvRect(pivotx2D-maxDis2D, rawPhoto->height- pivoty2D-maxDis2D, 2*maxDis2D, 2*maxDis2D));


	 //resize it to 512*512 for texture
	 IplImage *resized=cvCreateImage(cvSize(512, 512),8,3);
	 cvResize(rawPhoto, resized);
	 cvResetImageROI(rawPhoto);

	 /* Get texture data, translate from left top coord to left below coord*/

	 dataForModel=new BYTE[resized->height*resized->height*3];
	 for(int y=resized->height-1; y>=0; y--)
	 {
		 int y_oppos=resized->height-y-1;
		 BYTE *ptr=(BYTE*) (resized->imageData+y*resized->widthStep);
		 for(int x=0; x<resized->width; x++)
			{
				dataForModel[resized->widthStep*y_oppos+3*x]=ptr[3*(resized->width-1-x)+2];
				dataForModel[resized->widthStep*y_oppos+3*x+1]=ptr[3*(resized->width-1-x)+1];
				dataForModel[resized->widthStep*y_oppos+3*x+2]=ptr[3*(resized->width-1-x)+0];
			}
	 }


	 /* Set up texture environment and generate texture */
	 glutSetWindow(screen);
	 glEnable(GL_TEXTURE_2D);
	 glGenTextures(1, &textureForScr);
	 glBindTexture(GL_TEXTURE_2D, textureForScr);
	 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	 glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	 gluBuild2DMipmaps(GL_TEXTURE_2D, 3, resized->height, resized->height, GL_RGB, GL_UNSIGNED_BYTE, dataForModel);
	 cvReleaseImage(&resized);
	 delete[] dataForModel;
	 isModelTexGenerated=true;
}

void changeModel(char *modelFileName)
{
	model_unitized=glmReadOBJ(modelFileName);
	/* Save the vertices coordinates before unitized */
	originMod.clear();
	for(int i=0; i<=model_unitized->numvertices; i++)
	{
		morphPoint point;
		point.x=model_unitized->vertices[3*i];
		point.y=model_unitized->vertices[3*i+1];
		point.z=model_unitized->vertices[3*i+2];
		originMod.push_back(point);
	}
	/* Unitize the model and record its trans and scale paramaters */
	delete adjustPara;
	adjustPara=glmUnitize(model_unitized);

	glmFacetNormals(model_unitized);
	glmVertexNormals(model_unitized, 90.0);
	isModelTexGenerated=false;
	isDislandmarksGenerated=false;

}

void modelReset()
{
	//reset shape
	for(int i=0; i<originMod.size(); i++)
	{
		
		model_unitized->vertices[3*i]=originMod.at(i).x;
		model_unitized->vertices[3*i+1]=originMod.at(i).y;
		model_unitized->vertices[3*i+2]=originMod.at(i).z;
	}
	isModelUpdated=true;
	isModelTexGenerated=false;
}

void
drawmodel(void)
{
	/* Generate unitized model first */
	if (!model_unitized)
	{
		model_unitized = glmReadOBJ(".\\PickedPointsFaces\\reference face\\Reference_adjusted.obj");
		if (!model_unitized) exit(0);

		/* Save the vertices coordinates before unitized */
		originMod.clear();
		for(int i=0; i<=model_unitized->numvertices; i++)
		{
			morphPoint point;
			point.x=model_unitized->vertices[3*i];
			point.y=model_unitized->vertices[3*i+1];
			point.z=model_unitized->vertices[3*i+2];
			originMod.push_back(point);
		}
          /* Unitize the model and record its trans and scale paramaters */
		delete adjustPara;
		adjustPara=glmUnitize(model_unitized);

		glmFacetNormals(model_unitized);
		glmVertexNormals(model_unitized, 90.0);
		isModelTexGenerated=false;
		isDislandmarksGenerated=false;
	}
	/* IF updated recompute the model */
	else if(isModelUpdated==true)
	{
		/* Adjust transformed model using recorded parameters */
		unitizeAdjust(model_unitized->vertices, model_unitized->numvertices+1);
		glmFacetNormals(model_unitized);
		glmVertexNormals(model_unitized, 90.0);
		isModelUpdated=false;
	}
	if(isDrawModel==true)
	{
		if(isModelTexGenerated==false)
		{

			glmDraw(model_unitized, GLM_SMOOTH);

		}
		else
		{
			glutSetWindow(screen);
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, textureForScr);
			glmDraw(model_unitized, GLM_SMOOTH|GLM_TEXTURE);
			glDisable(GL_TEXTURE_2D);
		}
	}
	
}

void
identityMat(GLdouble m[16])
{
	m[0+4*0] = 1; m[0+4*1] = 0; m[0+4*2] = 0; m[0+4*3] = 0;
	m[1+4*0] = 0; m[1+4*1] = 1; m[1+4*2] = 0; m[1+4*3] = 0;
	m[2+4*0] = 0; m[2+4*1] = 0; m[2+4*2] = 1; m[2+4*3] = 0;
	m[3+4*0] = 0; m[3+4*1] = 0; m[3+4*2] = 0; m[3+4*3] = 1;
}


GLboolean
invert(GLdouble src[16], GLdouble inverse[16])
{
	double t;
	int i, j, k, swap;
	GLdouble tmp[4][4];

	identityMat(inverse);

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			tmp[i][j] = src[i*4+j];
		}
	}

	for (i = 0; i < 4; i++) {
		/* look for largest element in column. */
		swap = i;
		for (j = i + 1; j < 4; j++) {
			if (fabs(tmp[j][i]) > fabs(tmp[i][i])) {
				swap = j;
			}
		}

		if (swap != i) {
			/* swap rows. */
			for (k = 0; k < 4; k++) {
				t = tmp[i][k];
				tmp[i][k] = tmp[swap][k];
				tmp[swap][k] = t;

				t = inverse[i*4+k];
				inverse[i*4+k] = inverse[swap*4+k];
				inverse[swap*4+k] = t;
			}
		}

		if (tmp[i][i] == 0) {
			/* no non-zero pivot.  the matrix is singular, which
			shouldn't happen.  This means the user gave us a bad
			matrix. */
			return GL_FALSE;
		}

		t = tmp[i][i];
		for (k = 0; k < 4; k++) {
			tmp[i][k] /= t;
			inverse[i*4+k] /= t;
		}
		for (j = 0; j < 4; j++) {
			if (j != i) {
				t = tmp[j][i];
				for (k = 0; k < 4; k++) {
					tmp[j][k] -= tmp[i][k]*t;
					inverse[j*4+k] -= inverse[i*4+k]*t;
				}
			}
		}
	}
	return GL_TRUE;
}

float
normalize(float* v)
{
	float length;

	length = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	v[0] /= length;
	v[1] /= length;
	v[2] /= length;

	return length;
}

int spin_x=0;
int spin_y=0;
int spined_x=0;
int spined_y=0;
double translateDis=-13;


/************************************************************************/
/* functions for main window  */
/************************************************************************/
void
main_reshape(int width,  int height) 
{
	int tx, ty, tw, th;
	GLUI_Master.get_viewport_area( &tx, &ty, &tw, &th );
	glViewport( tx, ty, tw, th );
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(tx,tx+tw, ty+th, ty);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

#define GAP 10            /* gap between subwindows */
	sub_width = (tw-GAP*3);
	sub_height = (th-GAP*2);

	glutSetWindow(photoViewer);
	glutPositionWindow(tx+GAP, ty+GAP);
	glutReshapeWindow(sub_width/3, sub_width/3);

	double minWsize;
	if(2*sub_width/3>sub_height) minWsize=sub_height;
	else minWsize=2*sub_width/3;
	glutSetWindow(screen);
	glutPositionWindow(tx+2*GAP+sub_width/3,ty+ GAP);
	glutReshapeWindow(minWsize, minWsize);


	glutSetWindow(fileSelectorWnd);
	glutPositionWindow(tx+GAP, ty+sub_width/3+2*GAP);
	glutReshapeWindow(sub_width/3, sub_width/4);

	glutPostRedisplay();

}

void
main_display(void)
{
	glClearColor(0.8, 0.8, 0.8, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor3ub(0, 0, 0);
	setfont("helvetica", 12);
	glutSwapBuffers();
}
void
main_keyboard(unsigned char key, int x, int y)
{
	switch (key) {
	case 'p':
		mode = PERSPECTIVE;
		break;
	case 'o':
		mode = ORTHO;
		break;
	case 'f':
		mode = FRUSTUM;
		break;
	case '+':
		translateDis =translateDis +0.1;
		break;
	case '-':
		translateDis = translateDis - 0.1;
		break;
	case 'r':
		perspective[0].value = 60.0;
		perspective[1].value = 1.0;
		perspective[2].value = 1.0;
		perspective[3].value = 10.0;
		ortho[0].value = -1.0;
		ortho[1].value = 1.0;
		ortho[2].value = -1.0;
		ortho[3].value = 1.0;
		ortho[4].value = 1.0;
		ortho[5].value = 3.5;
		frustum[0].value = -1.0;
		frustum[1].value = 1.0;
		frustum[2].value = -1.0;
		frustum[3].value = 1.0;
		frustum[4].value = 1.0;
		frustum[5].value = 3.5;
		lookat[0].value = 0.0;
		lookat[1].value = 0.0;
		lookat[2].value = 2.0;
		lookat[3].value = 0.0;
		lookat[4].value = 0.0;
		lookat[5].value = 0.0;
		lookat[6].value = 0.0;
		lookat[7].value = 1.0;
		lookat[8].value = 0.0;
		translateDis=-4;
          spin_x=0;
		 spin_y=0;
		 spined_x=0;
		spined_y=0;
		break;


	case 27:
		exit(0);


	}

	redisplay_all();
}


/************************************************************************/
/* functions for screen window  */
/************************************************************************/

void
screen_reshape(int width, int height)
{
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (mode == PERSPECTIVE)
		gluPerspective(perspective[0].value, perspective[1].value, 
		perspective[2].value,30);
	else if (mode == ORTHO)
		glOrtho(ortho[0].value, ortho[1].value, ortho[2].value,
		ortho[3].value, ortho[4].value, ortho[5].value);
	else if (mode == FRUSTUM)
		glFrustum(frustum[0].value, frustum[1].value, frustum[2].value,
		frustum[3].value, frustum[4].value, frustum[5].value);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(lookat[0].value, lookat[1].value, lookat[2].value,
		lookat[3].value, lookat[4].value, lookat[5].value,
		lookat[6].value, lookat[7].value, lookat[8].value);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
}



void
screen_display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	glTranslated(0, translateDis, 0);
	glRotatef(spin_y, -1.0, 0.0, 0.0);
	glRotatef(spin_x, 0.0, 0.0, 1.0);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glColor3f(1.0,1.0, 1.0);

	drawmodel();
	
	if(isDrawSrcMarks==true)
	{
		glColor3f(1.0,0.0,0.0);
		glPointSize(5.0);
		glDisable(GL_LIGHTING);
		drawLandMarks(srcModelLandmark);
		glEnable(GL_LIGHTING);
	}
	if(isDrawDisMarks==true)
	{
		glColor3f(0.0,0.0,1.0);
		glPointSize(5.0);
		glDisable(GL_LIGHTING);
		drawLandMarks(disModelLandmark);
		glEnable(GL_LIGHTING);
	}
	glPopMatrix();
	glutSwapBuffers();
}

void
screen_menu(int value)
{
	char* name = 0;

	switch (value) {
	case 'a':
		genDisLandmarks();
		break;
	case 'b':
		forceAdjustDisLM();
		break;
	case'c':
		TPStransModel();
		break;
	case 'd':
		genTexForModel(transformedMod, disModelLandmark, featurePoints);
		break;
	case 'e':
		if(isDrawDisMarks==false) isDrawDisMarks=true;
		else isDrawDisMarks=false;
		break;
	case 'f':
		if(isDrawSrcMarks==false) isDrawSrcMarks=true;
		else isDrawSrcMarks=false;
		break;
	}

	redisplay_all();
}

int old_x, old_y;

void
screen_mouse(int button, int state, int x, int y)
{
	/* reset the start position */
	old_x = x;
	old_y = y;

	/* record the degrees already spinned */
	spined_x=spin_x%360;
	spined_y=spin_y%360;

	redisplay_all();
}

void
screen_motion(int x, int y)
{
	spin_x = (x - old_x+spined_x)%360;
	spin_y = (y - old_y+spined_y)%360;

	redisplay_all();
}


/************************************************************************/
/*  functions for photoViewer window                                                                      */
/************************************************************************/

/* function for feature points generation  */

void ASMpreparation()
{
	ASM::initASM();
	isASMSetted=true;
}

/*load img to raw photo and find feature points */
void loadPicture(const char *filePath)
{
	if(isASMSetted==true)
	{
		isFeaturePointsFounded=false;
		cvReleaseImage(&rawPhoto);
		rawPhoto=cvLoadImage(filePath, CV_LOAD_IMAGE_COLOR);
		int nchannel=rawPhoto->nChannels;
		int collor=rawPhoto->depth;
		IplImage *RGB=cvCreateImage(cvSize(rawPhoto->width, rawPhoto->height), 8, 3);
		cvCvtColor(rawPhoto, RGB, CV_BGR2RGB);


		/* erase all the feature points already stored first */
		featurePoints.clear();
		ASM::getASMPoints(filePath, &featurePoints.pointsSet, featurePoints.width, featurePoints.height);

		/* average points at mouth */
		featurePoints.pointsSet.at(60).x=(featurePoints.pointsSet.at(60).x+featurePoints.pointsSet.at(65).x)/2;
		featurePoints.pointsSet.at(60).y=(featurePoints.pointsSet.at(60).y+featurePoints.pointsSet.at(65).y)/2;
		featurePoints.pointsSet.at(61).x=featurePoints.pointsSet.at(66).x;
		featurePoints.pointsSet.at(61).y=featurePoints.pointsSet.at(66).y;
		featurePoints.pointsSet.at(62).x=(featurePoints.pointsSet.at(62).x+featurePoints.pointsSet.at(63).x)/2;
		featurePoints.pointsSet.at(62).y=(featurePoints.pointsSet.at(62).y+featurePoints.pointsSet.at(63).y)/2;
		featurePoints.pointsSet.at(63).x=featurePoints.pointsSet.at(67).x;
		featurePoints.pointsSet.at(63).y=featurePoints.pointsSet.at(67).y;
		//pop the last four points.
		featurePoints.pointsSet.pop_back();
		featurePoints.pointsSet.pop_back();
		featurePoints.pointsSet.pop_back();
		featurePoints.pointsSet.pop_back();

		//add 8 points on ear
		double faceWidth=abs(featurePoints.pointsSet.at(1).x-featurePoints.pointsSet.at(13).x);
		ASM::ASMPoint tPoint;
		tPoint.y=0.5*(featurePoints.pointsSet.at(13).y+featurePoints.pointsSet.at(14).y);
		tPoint.x=featurePoints.pointsSet.at(13).x;
		tPoint.idx=64;
		featurePoints.pointsSet.push_back(tPoint);
		tPoint.y=0.5*(featurePoints.pointsSet.at(12).y+featurePoints.pointsSet.at(11).y);
		tPoint.x=featurePoints.pointsSet.at(13).x;
		tPoint.idx=65;
		featurePoints.pointsSet.push_back(tPoint);
		tPoint.y=0.5*(featurePoints.pointsSet.at(13).y+featurePoints.pointsSet.at(12).y);
		tPoint.x=featurePoints.pointsSet.at(13).x;
		tPoint.idx=66;
		featurePoints.pointsSet.push_back(tPoint);
		tPoint.y=0.5*(featurePoints.pointsSet.at(0).y+featurePoints.pointsSet.at(1).y);
		tPoint.x=featurePoints.pointsSet.at(1).x;
		tPoint.idx=67;
		featurePoints.pointsSet.push_back(tPoint);
		tPoint.y=0.5*(featurePoints.pointsSet.at(2).y+featurePoints.pointsSet.at(3).y);
		tPoint.x=featurePoints.pointsSet.at(1).x;
		tPoint.idx=68;
		featurePoints.pointsSet.push_back(tPoint);
		tPoint.y=0.5*(featurePoints.pointsSet.at(1).y+featurePoints.pointsSet.at(2).y);
		tPoint.x=featurePoints.pointsSet.at(1).x;
		tPoint.idx=69;
		featurePoints.pointsSet.push_back(tPoint);
		tPoint.y=0.5*(featurePoints.pointsSet.at(0).y+featurePoints.pointsSet.at(1).y);
		tPoint.x=featurePoints.pointsSet.at(1).x-0.15*faceWidth;
		tPoint.idx=70;
		featurePoints.pointsSet.push_back(tPoint);
		tPoint.y=0.5*(featurePoints.pointsSet.at(13).y+featurePoints.pointsSet.at(14).y);
		tPoint.x=featurePoints.pointsSet.at(13).x+0.15*faceWidth;
		tPoint.idx=71;
		featurePoints.pointsSet.push_back(tPoint);



		
		


		/*get bounds of this point set*/
		featurePoints.max_x=(std::numeric_limits<double>::min)();
		featurePoints.max_y=(std::numeric_limits<double>::min)();
		featurePoints.min_x=(std::numeric_limits<double>::max)();
		featurePoints.min_y=(std::numeric_limits<double>::max)(); 
		for(int i=0; i<featurePoints.pointsSet.size(); i++)
		{
			double this_x=featurePoints.pointsSet.at(i).x;
			double this_y=featurePoints.pointsSet.at(i).y;
			if(this_x>featurePoints.max_x)
			{
				featurePoints.max_x=this_x;
			}
			if(this_x<featurePoints.min_x)
			{
				featurePoints.min_x=this_x;
			}
			if(this_y>featurePoints.max_y)
			{
				featurePoints.max_y=this_y;
			}
			if(this_y<featurePoints.min_y)
			{
				featurePoints.min_y=this_y;
			}
		}
		

		/* set the pivotIdx number*/ 
		featurePoints.pivotIdx=63;
		isFeaturePointsFounded=true;

	}	
}


/* generate texture and find the coordinates in texture that to be matched to polygon in the future */
void genTexForPV()
{
	if(isFeaturePointsFounded==true)
	{
		

		/* Get face region coord First */
		double maxWidth=featurePoints.max_x-featurePoints.min_x;
		double maxHeight=featurePoints.max_y-featurePoints.min_y;
		double upMargin=maxHeight*0.05;
		double downMargin=maxHeight*0.05;
		double leftMargin=maxWidth*0.05;
		double rightMargin=maxWidth*0.05;


     
		double texLeastWidth=maxWidth+leftMargin+rightMargin;
		double texLeastHeight=maxHeight+upMargin+downMargin;
		double squareLength;
		if(texLeastWidth>texLeastHeight)
		{
			faceRegionWidth=texLeastWidth;
			faceRegionHeight=texLeastWidth;
			squareLength=texLeastWidth;
		}
		else
		{
			faceRegionWidth=texLeastHeight;
			faceRegionHeight=texLeastHeight;
			squareLength=texLeastHeight;
		}

		texCoordForPV[0]=featurePoints.min_x-(squareLength-maxWidth)/2;
		texCoordForPV[1]=featurePoints.min_y-(squareLength-maxHeight)/2;
		//note: transfer from below left from top left
		CvPoint2D32f center;
		center.x=texCoordForPV[0]+squareLength/2;
		center.y=rawPhoto->height-(texCoordForPV[1]+squareLength/2);

		//cut the region face for texture
		cvSetImageROI(rawPhoto,cvRect(center.x-squareLength/2,center.y-squareLength/2,squareLength,squareLength));

		//resize it to 512*512 for texture
		IplImage *resized=cvCreateImage(cvSize(512, 512),8,3);
		cvResize(rawPhoto, resized);
		cvResetImageROI(rawPhoto);

		/* Get texture data, translate from left top coord to left below coord*/
		dataForPV=new BYTE[512*512*3];
		for(int y=resized->height-1; y>=0; y--)
		{
			int y_oppos=resized->height-y-1;
			BYTE *ptr=(BYTE*) (resized->imageData+y*resized->widthStep);
			for(int x=0; x<resized->width; x++)
			{
				dataForPV[resized->widthStep*y_oppos+3*x]=ptr[3*x+2];
				dataForPV[resized->widthStep*y_oppos+3*x+1]=ptr[3*x+1];
				dataForPV[resized->widthStep*y_oppos+3*x+2]=ptr[3*x+0];
			}
		}
		cvReleaseImage(&resized);

		/* Set up texture environment and generate texture */
		glutSetWindow(photoViewer);
		if(isPVTexGenerated==false)glGenTextures(1, &textureForPV); //if textureforPV is not generated before, generate it
          glBindTexture(GL_TEXTURE_2D, textureForPV);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		gluBuild2DMipmaps(GL_TEXTURE_2D, 3, 512, 512, GL_RGB, GL_UNSIGNED_BYTE, dataForPV);
		delete[] dataForPV;
		isPVTexGenerated=true;

		/*
		glutSetWindow(screen);
				glGenTextures(1, &textureForScr);
				glBindTexture(GL_TEXTURE_2D, textureForPV);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
				glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
				gluBuild2DMipmaps(GL_TEXTURE_2D, 3, 512, 512, GL_RGB, GL_UNSIGNED_BYTE, data);*/
		
		

	}
}


/* generate coordinates of polygon vertex for texture mapping */
void getVertexCoordFitWnd()
{
	  isVertexCoordGenerated=false;
			/* vertex 1 */
			vertexCoordForPV[0]=0;
			vertexCoordForPV[1]=0;
			/* vertex 2 */
			vertexCoordForPV[2]=PVmin;
			vertexCoordForPV[3]=0;
			/* vertex 3 */
			vertexCoordForPV[4]=PVmin;
			vertexCoordForPV[5]=PVmin;
			/* vertex 4 */
			vertexCoordForPV[6]=0;
			vertexCoordForPV[7]=PVmin;
		polygonWidth=PVmin;
		polygonHeight=PVmin;
	isVertexCoordGenerated=true;

}


/*  Functions for photo texture mapping  */
void texMappingForPV()
{
	if(isPVTexGenerated==true&&isVertexCoordGenerated==true)
	{
		glBindTexture(GL_TEXTURE_2D, textureForPV);
		glEnable(GL_TEXTURE_2D);
		glBegin(GL_POLYGON);
		     glTexCoord2f(0, 0.0);
			glVertex2d(vertexCoordForPV[0], vertexCoordForPV[1]);
			glTexCoord2f(1.0, 0.0);
			glVertex2d(vertexCoordForPV[2], vertexCoordForPV[3]);
			glTexCoord2f(1.0, 1.0);
			glVertex2d(vertexCoordForPV[4], vertexCoordForPV[5]);
			glTexCoord2f(0, 1.0);
			glVertex2d(vertexCoordForPV[6], vertexCoordForPV[7]);
		glEnd();
		glDisable(GL_TEXTURE_2D);
	}
	/* if texture not generated just draw the polygon*/
	else if ( isVertexCoordGenerated==true)
	{
		glBegin(GL_POLYGON);
		glVertex2d(vertexCoordForPV[0], vertexCoordForPV[1]);
		glVertex2d(vertexCoordForPV[2], vertexCoordForPV[3]);
		glVertex2d(vertexCoordForPV[4], vertexCoordForPV[5]);
		glVertex2d(vertexCoordForPV[6], vertexCoordForPV[7]);
		glEnd();
	}
}

/* Convert texture coord to PV window coord */
void coordImg2Wnd(const double x_src, const double y_src, double &x_wnd, double &y_wnd )
{
     x_wnd=(x_src - texCoordForPV[0])*(polygonWidth/faceRegionWidth);
	y_wnd=(y_src - texCoordForPV[1])*(polygonHeight/faceRegionHeight);

}

/* Convert PV window coord to texture coord*/
void coordWnd2Img(const double x_src, const double y_src, double &x_tex, double &y_tex )
{

	x_tex=x_src*(faceRegionWidth/polygonWidth)+texCoordForPV[0];
	y_tex=(faceRegionHeight-y_src*(faceRegionHeight/polygonHeight))+texCoordForPV[1];
}

void drawFeaturePoints()
{
	if(isVertexCoordGenerated==true && featurePoints.pointsSet.size() > 8)
	{
		//draw non ear points red
		glColor3f(1.0,0.0,0.0);
		glPointSize(4.0);
		glBegin(GL_POINTS);
		for(int i=0; i<featurePoints.pointsSet.size()-8; i++)
		{
			if(i!=pickedIdx)
			{
				double x_src=featurePoints.pointsSet.at(i).x;
				double y_src=featurePoints.pointsSet.at(i).y;
				double x_wnd, y_wnd;
				coordImg2Wnd(x_src, y_src, x_wnd, y_wnd);
				glVertex2d(x_wnd, y_wnd);
			}
		}
		glEnd();
		//draw the ear points in green
		glColor3f(0.0,1.0,0.0);
		glBegin(GL_POINTS);
		double size=featurePoints.pointsSet.size();
		for(int i=0; i<8; i++)
		{
			int cIdx=size-i-1;
			if(cIdx!=pickedIdx)
			{
				double x_src=featurePoints.pointsSet.at(cIdx).x;
				double y_src=featurePoints.pointsSet.at(cIdx).y;
				double x_wnd, y_wnd;
				coordImg2Wnd(x_src, y_src, x_wnd, y_wnd);
				glVertex2d(x_wnd, y_wnd);
			}
		}
		glEnd();

		//draw the picked Point in a different color.
		if(pickedIdx>=0)
		{
			glColor3f(1.0,1.0,0.0);
			glBegin(GL_POINTS);
			double x_src=featurePoints.pointsSet.at(pickedIdx).x;
			double y_src=featurePoints.pointsSet.at(pickedIdx).y;
			double x_wnd, y_wnd;
			coordImg2Wnd(x_src, y_src, x_wnd, y_wnd);
			glVertex2d(x_wnd, y_wnd);
			glEnd();
		}

	}
}

void linkFeaturePoints()
{
	if(isVertexCoordGenerated)
	{
		
		//link all ear points blue
		glColor3f(0.0,0.0,1.0);
		glLineWidth(2.0);
		glBegin(GL_LINE_STRIP);
		for(int i=0; i<featurePoints.pointsSet.size()-8; i++)
		{
			if(i!=pickedIdx)
			{
				double x_src=featurePoints.pointsSet.at(i).x;
				double y_src=featurePoints.pointsSet.at(i).y;
				double x_wnd, y_wnd;
				coordImg2Wnd(x_src, y_src, x_wnd, y_wnd);
				glVertex2d(x_wnd, y_wnd);

			}
		}
		glEnd();

	}
}

/* regular OpenGL functions for PV window */
void
photoViewer_reshape(int width, int height)
{
	int tx, ty, tw, th;
	GLUI_Master.get_viewport_area( &tx, &ty, &tw, &th );
	PVwidth=tw;
	PVheight=th;
	
	if(PVwidth>PVheight)PVmin=PVheight;
	else PVmin=PVwidth;

	getVertexCoordFitWnd();
	glViewport(tx, ty, PVmin, PVmin);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glClearColor(0.2, 0.2, 0.2, 1.0);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_LIGHT0);

}


void
photoViewer_display(void)
{
    /* Set up model */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, PVmin, 0, PVmin, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);

    /* If texture already generated ,mapping it to polygon*/
	glColor3f(0.0,0.0,0.0);

	if(isVertexCoordGenerated)
	{
		texMappingForPV();
	}

	if(isLinkFeaturePoints==true)
	{
		linkFeaturePoints();
	}

	if(isDrawFeaturePoints==true)
	{
		drawFeaturePoints();
	}

	glDisable(GL_TEXTURE_2D);
	glutSwapBuffers();
}

void
photoViewer_menu(int value)
{
	switch (value) {
	case 'a':
		{
			/* reset all indicaters */
			isFeaturePointsFounded=false; //indicate whether feature points are founded;
			isPVTexGenerated=false; //indicated whether texture has been generated
			isVertexCoordGenerated=false; //indicate whether the polygon vertex has been determined

			/* keep the rawphoto pointer but clear the memory it takes*/
			IplImage *temp=rawPhoto;
			cvReleaseImage(&temp);

			/* Generate texture from image and waiting to be mapped */
			//loadPicture("E:\\document\\My Pictures\\gt_db\\s01\\01.jpg");
			genTexForPV();
			break;
		}
	case 'b':
		break;
	}

	redisplay_all();
}

double dis(double x1, double y1, double x2, double y2)
{
	return std::sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

void
photoViewer_mouse(int button, int action, int x, int y)
{
	if(isFeaturePointsFounded==true)
	{
		if(button==GLUT_LEFT_BUTTON&&action==GLUT_DOWN)
		{
			double img_x, img_y;
			coordWnd2Img(x, y, img_x, img_y);
			//finding the closest feature point
			double mindis=dis(img_x, img_y, featurePoints.pointsSet.at(0).x, featurePoints.pointsSet.at(0).y);
			int idx=0;
			for(int i=0 ; i<featurePoints.pointsSet.size(); i++)
			{
				double cdis=dis(img_x, img_y, featurePoints.pointsSet.at(i).x, featurePoints.pointsSet.at(i).y);
				if(mindis>cdis)
				{
					mindis=cdis;
					idx=i;
				}
			}
			if(mindis<disThreshold)
			{
				pickedIdx=idx;
				featurePoints.pointsSet.at(pickedIdx).x=x;
				featurePoints.pointsSet.at(pickedIdx).y=y;
			}
		}
		if(button==GLUT_LEFT_BUTTON&&action==GLUT_UP)
		{
			if(pickedIdx>=0)
			{
				double img_x, img_y;
				coordWnd2Img(x, y, img_x, img_y);
				//update featue points
				featurePoints.pointsSet.at(pickedIdx).x=img_x;
				featurePoints.pointsSet.at(pickedIdx).y=img_y;
				pickedIdx=-1;
			}
		}
		redisplay_all();
	}
}



void
photoViewer_motion(int x, int y)
{
	if(isFeaturePointsFounded==true)
	{
		if(pickedIdx>=0)
		{
			double img_x, img_y;
			coordWnd2Img(x, y, img_x, img_y);
			//update featue points
			featurePoints.pointsSet.at(pickedIdx).x=img_x;
			featurePoints.pointsSet.at(pickedIdx).y=img_y;
		}
		redisplay_all();
	}
	
}

/************************************************************************/
/*  Function for FileSelectorWnd                                                                     */
/************************************************************************/
void fileSelectorWnd_reshape(int width, int height)
{

}

void fileSelectorWnd_display()
{
	glClearColor(0.8, 0.8, 0.8, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor3ub(0, 0, 0);
	setfont("helvetica", 12);
	glutSwapBuffers();
}
void control_cb( int control )
{
	switch(control)
	{
	case LOAD_IMG:
		{
			/* prepare for file browser */
			ZeroMemory( &ofn , sizeof( ofn));
			ofn.lStructSize = sizeof ( ofn );
			ofn.hwndOwner = NULL  ;
			ofn.lpstrFile = szFile ;
			ofn.lpstrFile[0] = '\0';
			ofn.nMaxFile = sizeof( szFile );
			ofn.lpstrFilter = "All\0*.*\0Text\0*.TXT\0";
			ofn.nFilterIndex =1;
			ofn.lpstrFileTitle = NULL ;
			ofn.nMaxFileTitle = 0 ;
			ofn.lpstrInitialDir=NULL ;
			ofn.Flags = OFN_PATHMUSTEXIST|OFN_FILEMUSTEXIST ;

			/* reading photo */
			if(GetOpenFileNameA( &ofn ))
			{
				/* Draw Source Image First*/
				IplImage *srcImg=cvLoadImage(szFile);
				cvShowImage("Source Image Viewer",srcImg);
				cvReleaseImage(&srcImg);

				/* reset all indicator for PV */
				isFeaturePointsFounded=false; //indicate whether feature points are founded;
				isPVTexGenerated=false; //indicated whether texture for Photo Viewer has been generated
				isVertexCoordGenerated=false; //indicate whether the polygon vertex has been determined
				isDislandmarksGenerated=false;//indicate whether dislandmarks is generated

				/* generate texture */
				std::cout<<"ASM Searching..."<<endl;
				loadPicture(szFile);
				genTexForPV();
				std::cout<<"ASM Search Ready"<<endl;
			}
			break;
		}

	case  SELECT_MODEL:
		{
			break;

		}
	case  GEN_DISLANDMARKS:
		{
			genDisLandmarks();
			break;

		}
	case  COMPULSORY_ADJUST:
		{
			forceAdjustDisLM();
			break;

		}
	case TPS_TRANSFORM:
		{
			std::cout<<"TPS Transforming... Please Wait..."<<endl;
			TPStransModel();
			std::cout<<"TPS Transform Ready."<<endl;
			isDrawModel=true;

			break;
			
		}
	case  TEXTURE_MAPPING:
		{
			genTexForModel(transformedMod, disModelLandmark, featurePoints);
			break;
		}
	case MODEL_RESET:
		{
			modelReset();
			break;
		}

	}
	redisplay_all();
}

/************************************************************************/
/*  refresh function                                                                     */
/************************************************************************/
void
redisplay_all(void)
{
	double minWsize;
	if(2*sub_width/3>sub_height)minWsize=sub_height;
	else minWsize=2*sub_width/3;
	glutSetWindow(screen);
	screen_reshape(minWsize, minWsize);
	glutPostRedisplay();

	glutSetWindow(photoViewer);
	photoViewer_reshape(sub_width/3, sub_width/3);
	glutPostRedisplay();

	glutSetWindow(fileSelectorWnd);
	fileSelectorWnd_reshape(sub_width/3, sub_width/4);
	glutPostRedisplay();
}

/* perform functions unrelated to graphics tasks */
void modelViewerInit()
{
	/*establish ASM models  */
	ASMpreparation();
	morph::loadAllModels(_T(".\\PickedPointsFaces\\"),&landmarkSet, &densePointsSet);
	srcModelLandmark.clear();
	morph::readControlPointsFromPP(srcModelLandmark, _T(".\\PickedPointsFaces\\Reference_adjusted.pp"));
	//cvNamedWindow("Source Image Viewer");

}
/************************************************************************/
/* main function                                                                     */
/************************************************************************/
int _tmain(int argc, char** argv)
{
	/*preparation */
	modelViewerInit();

	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(1024+GAP*3, 512+GAP*2);
	glutInitWindowPosition(200, 50);
	glutInit(&argc, argv);

	window = glutCreateWindow("3D Face Reconstructor - Yiding Chai");
	glutDisplayFunc(main_display);
	glutKeyboardFunc(main_keyboard);
	GLUI_Master.set_glutReshapeFunc( main_reshape );  
	

	/*** Create the side subwindow ***/
	glui = GLUI_Master.create_glui_subwindow( window, 
		GLUI_SUBWINDOW_RIGHT );

	new GLUI_Button(glui, "Model Select", SELECT_MODEL, control_cb);
	new GLUI_Button(glui, "Model Reset", MODEL_RESET, control_cb);

	obj_panel = new GLUI_Rollout(glui, "Model Processing Kit", true );


	/***** Button for processing Model *****/

	new GLUI_Button( obj_panel, "Gen DisLandmarks", GEN_DISLANDMARKS, control_cb);
	new GLUI_Button(obj_panel, "Compulsory Adjust", COMPULSORY_ADJUST, control_cb);
	new GLUI_Button(obj_panel, "TPS Transform", TPS_TRANSFORM, control_cb);
	new GLUI_Button(obj_panel, "Texture Mapping", TEXTURE_MAPPING, control_cb);

	obj_panel2 =new GLUI_Rollout(glui, "Draw Kit", false);

	/******CheckBoxes for control 3D drawing***/
	new GLUI_Checkbox(obj_panel2, "Draw Model",&isDrawModel, 1, control_cb);
	new GLUI_Checkbox(obj_panel2, "Draw SrcLandmarks",&isDrawSrcMarks, 1, control_cb);
	new GLUI_Checkbox(obj_panel2, "Draw DisLandmarks",&isDrawDisMarks, 1, control_cb);

	glui->set_main_gfx_window( window );
	GLUI_Master.set_glutIdleFunc( myGlutIdle );

	double minWsize;
	if(2*sub_width/3>sub_height) minWsize=sub_height;
	else minWsize=2*sub_width/3;


	screen = glutCreateSubWindow(window, sub_width/3+2*GAP, GAP, minWsize, minWsize);
	glutReshapeFunc(screen_reshape);
	glutDisplayFunc(screen_display);
	glutKeyboardFunc(main_keyboard);
	glutMouseFunc(screen_mouse);
	glutMotionFunc(screen_motion);
	glutCreateMenu(screen_menu);
	glutAddMenuEntry("Gen DisLandmarks", 'a');
	glutAddMenuEntry("Force Adjust",'b');
     glutAddMenuEntry("TPS", 'c');
     glutAddMenuEntry("Texture Mapping", 'd');
	glutAddMenuEntry("Show DisLandmarks", 'e');
	glutAddMenuEntry("Show SrcLandmarks", 'f');
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	photoViewer=glutCreateSubWindow(window, GAP, GAP, sub_width/3, sub_width/3 );
	GLUI_Master.set_glutReshapeFunc(photoViewer_reshape);
	glutDisplayFunc(photoViewer_display);
	glutMouseFunc(photoViewer_mouse);
	glutMotionFunc(photoViewer_motion);
	glutCreateMenu(photoViewer_menu);
	glutAddMenuEntry("test_image",'a');
	glutAddMenuEntry("just redisplay", 'b');
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	/*** Create the side subwindow ***/
	fileSelectorWnd=glutCreateSubWindow(window, GAP, sub_width/3+2*GAP,   sub_width/3, sub_width/6);
	GLUI_Master.set_glutReshapeFunc(fileSelectorWnd_reshape);
	glutDisplayFunc(fileSelectorWnd_display);
     fileBrowserGlui = GLUI_Master.create_glui_subwindow(fileSelectorWnd,GLUI_SUBWINDOW_TOP);
	new GLUI_Button( fileBrowserGlui, "Open Photo", 300, control_cb );
	GLUI_Checkbox dfp( fileBrowserGlui, "Draw Feature Points", &isDrawFeaturePoints, 301, control_cb );
	GLUI_Checkbox lfp( fileBrowserGlui, "Link Feature Points", &isLinkFeaturePoints, 302, control_cb );
	fileBrowserGlui->set_main_gfx_window(fileSelectorWnd);

	redisplay_all();

	glutMainLoop();

	exit(0);
	
}

