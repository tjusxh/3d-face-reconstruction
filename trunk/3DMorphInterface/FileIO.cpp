#include "stdafx.h"
#include "FileIO.h"
using namespace  std;

LPTSTR k;
GLMmodel * readGLMfromPLY(char *PLYpath)
{
	GLMmodel *model;
	/* allocate a new model */
	model = (GLMmodel*)malloc(sizeof(GLMmodel));
	model->pathname    = strdup(PLYpath);
	model->mtllibname    = NULL;
	model->numvertices   = 0;
	model->vertices    = NULL;
	model->numnormals    = 0;
	model->normals     = NULL;
	model->numtexcoords  = 0;
	model->texcoords       = NULL;
	model->numfacetnorms = 0;
	model->facetnorms    = NULL;
	model->numtriangles  = 0;
	model->triangles       = NULL;
	model->nummaterials  = 0;
	model->materials       = NULL;
	model->numgroups       = 0;
	model->groups      = NULL;
	model->position[0]   = 0.0;
	model->position[1]   = 0.0;
	model->position[2]   = 0.0;

	/* add only group to GLMmodel */
	GLMgroup* group= (GLMgroup*)malloc(sizeof(GLMgroup));
	group->name = strdup("only one");
	group->material = 0;
	group->numtriangles = 0;
	group->triangles = NULL;
	group->next = model->groups;
	model->groups = group;
	model->numgroups++;
	/* allocate memory */




	/* start to reading PLY file */ 
	ifstream inputFile(PLYpath);
	if(inputFile.is_open())
	{
		int status=READING_HEADER;
		int pointsNum=0;
		int countVertex=1;
		int countTriangle=0;
		while(!inputFile.eof())
		{
			char line[1000];
			inputFile.getline(line,1000);
			string s(line);

			switch(status)
			{

				/* just skip header. */
			case READING_HEADER:
				{
					//record vertex number.
					if(s.find("element vertex")!=-1)
					{
						model->numvertices=atol(s.substr(15,s.length()-15).c_str());
						model->vertices = (GLfloat*)malloc(sizeof(GLfloat) *
							3 * (model->numvertices + 1));
					}
					/*  record triangles number */
					else if(s.find("element face")!=-1)
					{
						model->numtriangles=atol(s.substr(13,s.length()-13).c_str());
						model->triangles = (GLMtriangle*)malloc(sizeof(GLMtriangle) *
							model->numtriangles);
						model->groups->numtriangles=model->numtriangles;
						model->groups->triangles=new GLuint[model->numtriangles];
						if (model->numnormals) {
							model->normals = (GLfloat*)malloc(sizeof(GLfloat) *
								3 * (model->numnormals + 1));
						}
						if (model->numtexcoords) {
							model->texcoords = (GLfloat*)malloc(sizeof(GLfloat) *
								2 * (model->numtexcoords + 1));
						}
					}
					else if(s.find("end_header")!=-1)
					{
						status=READING_VERTEX;
					}
					break;
				}

					/* if reading vertex right now, then using TPS convert the vertex coordinates. */ 
			case READING_VERTEX:
				{
					//eat up all empty lines
					if(s.size()==0&&countVertex==1)
					{					
					}
					else if(countVertex<model->numvertices+1)
					{

						double coord[3];
						getXYZPLY(s,coord);
						model->vertices[countVertex*3]=coord[0];
						model->vertices[countVertex*3+1]=coord[1];
						model->vertices[countVertex*3+2]=coord[2];
						countVertex++;
					}
					else if(countVertex==model->numvertices+1)
					{
						status=READING_POLY;
					}
					break;					 
				}
				
				case READING_POLY:
				{
					//eat up all empty lines
					if(s.size()==0&&countVertex==0){}
					else if(countTriangle<model->numtriangles)
					{
						unsigned int verticesIdx[3];
						getTriangleFacesPLY(s,verticesIdx);
						model->triangles[countTriangle].vindices[0]=verticesIdx[0]+1;
						model->triangles[countTriangle].vindices[1]=verticesIdx[1]+1;
						model->triangles[countTriangle].vindices[2]=verticesIdx[2]+1;
						model->groups->triangles[countTriangle]=countTriangle;
						model->triangles[countTriangle].findex=countTriangle;
						countTriangle++;
					}
				}
			}
		}
		return model;
	}
	else return NULL;
}
//read control points from point file ".pp"
void readControlPointsFromPP(std::vector<Vec>* points, LPTSTR path)
{
	cout<<"reading landmarks from: "<<(*path)<<"..."<<endl;
	int status=0;
	std::vector<Vec>::iterator iter;
	ifstream myfile (path);
	if (myfile.is_open())
	{
		char line[1024]={0};
		while (! myfile.eof() )
		{
			myfile.getline(line,100);
			string s(line);
			if(s.find("<point ")!=-1)
			{
				double coord[3];
				getXYZPP(s,coord);
				Vec v(coord[0], coord[1], coord[2]);
				points->push_back(v);
			}
		}
		myfile.close();
	}
	else cout << "Unable to open file"; 
}

/*
void readControlPointsFromPP(ap::real_1d_array &controlPoints, LPTSTR path)
{
	cout<<"reading landmarks from: "<<(*path)<<"..."<<endl;
	int status=0;
	std::vector<Vec> points;
	std::vector<Vec>::iterator iter;
	ifstream myfile (path);
	if (myfile.is_open())
	{
		char line[1024]={0};
		while (! myfile.eof() )
		{
			myfile.getline(line,100);
			string s(line);
			if(s.find("<point ")!=-1)
			{
				double coord[3];
				getXYZPP(s,coord);
				Vec v(coord[0], coord[1], coord[2]);
				points.push_back(v);
			}
		}
		myfile.close();
		controlPoints.setlength(3*points.size());
		for(int i=0; i<points.size(); i++)
		{
			controlPoints(i*3)=points.at(i).x;
			controlPoints(i*3+1)=points.at(i).y;
			controlPoints(i*3+2)=points.at(i).z;
		}
	}
	else cout << "Unable to open file"; 
}*/


void writeControlPointsToPP(std::vector<Vec>*points, LPTSTR targetPath, LPTSTR refPath)
{
	cout<<"writing landmarks to" <<(*targetPath)<<"..."<<endl;
	std::vector<Vec>:: iterator iter;
	ifstream inputFile(refPath);
	ofstream outputFile(targetPath);
	if(inputFile.is_open())
	{
		char line[1024]={0};
		//copy refFile until the points data starts
		while (! inputFile.eof() )
		{
			inputFile.getline(line,100);
			string s(line);
			if(s.find("<point ")!=-1)
			{
				break;
			}
			else
				outputFile<<s<<endl;
		}
		//write points data 
		for(int i=0; i<points->size(); i++)
		{
			outputFile<<"<point x="<<'"'<<points->at(i).x<<'"'
				<<" y="<<'"'<<points->at(i).y<<'"'
				<<" z="<<'"'<<points->at(i).z<<'"'
				<<" active="<<'"'<<"0"<<'"'
				<<" name="<<'"'<<i<<'"'
				<<" />"
				<<endl;
		}
		outputFile<<"</PickedPoints>"<<endl;


	}
}

/*
void writeControlPointsToPP(ap::real_1d_array &controlPoints, LPTSTR targetPath, LPTSTR refPath)
{
	cout<<"writing landmarks to" <<(*targetPath)<<"..."<<endl;
	std::vector<Vec>:: iterator iter;
	ifstream inputFile(refPath);
	ofstream outputFile(targetPath);
	if(inputFile.is_open())
	{
		char line[1024]={0};
		//copy refFile until the points data starts
		while (! inputFile.eof() )
		{
			inputFile.getline(line,100);
			string s(line);
			if(s.find("<point ")!=-1)
			{
				break;
			}
			else
				outputFile<<s<<endl;
		}
		//write points data 
		int numOfPoints=controlPoints.gethighbound()/3;
		for(int i=0; i<numOfPoints; i++)
		{
			outputFile<<"<point x="<<'"'<<controlPoints(3*i)<<'"'
				<<" y="<<'"'<<controlPoints(3*i+1)<<'"'
				<<" z="<<'"'<<controlPoints(3*i+2)<<'"'
				<<" active="<<'"'<<"0"<<'"'
				<<" name="<<'"'<<i<<'"'
				<<" />"
				<<endl;
		}
		outputFile<<"</PickedPoints>"<<endl;


	}
}*/

//extract XYZ coordinates from string s in points file
void getXYZPP(string s, double* coord )
{
	int start=s.find('"');
	int end=s.find('"' , start+1);
	coord[0]=atof(s.substr(start+1,end-start).c_str());
	start=s.find('"', end+1);
	end=s.find('"', start+1);
	coord[1]=atof(s.substr(start+1,end-start).c_str());
	start=s.find('"', end+1);
	end=s.find('"', start+1);
	coord[2]=atof(s.substr(start+1,end-start).c_str());
}

void readPointsFromPLY(mrpt::slam::CSimplePointsMap *points, LPTSTR densePointsPath)
{
	cout<<"reading points from: "<<(*densePointsPath)<<"..."<<endl;
	ifstream inputFile(densePointsPath);
	if(inputFile.is_open())
	{
		int status=READING_HEADER;
		int pointsNum=0;
		int count=0;
		while(!inputFile.eof())
		{
			char line[1000];
			inputFile.getline(line,1000);
			string s(line);

			switch(status)
			{

				//just skip header.
			case READING_HEADER:
				{
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

				//if reading vertex right now, then using TPS convert the vertex coordinates. 
			case READING_VERTEX:
				{
					//eat up all empty lines
					if(s.size()==0&&count==0)
					{					
					}
					else if(count<pointsNum)
					{
						count++;
						double coord[3];
						getXYZPLY(s,coord);
						points->insertPoint(coord[0], coord[1], coord[2]);
					}
					else if(count==pointsNum)
					{
						status=READING_POLY;
					}
					break;					 
				}
			}
			//all vertex have been recorded just exit.
			if(status==READING_POLY)
			{
				break;
			}
		}
	}
}

void readPointsFromPLY(std::vector<Vec> *points, LPTSTR densePointsPath)
{
	cout<<"reading points from: "<<(*densePointsPath)<<"..."<<endl;
	ifstream inputFile(densePointsPath);
	if(inputFile.is_open())
	{
		int status=READING_HEADER;
		int pointsNum=0;
		int count=0;
		while(!inputFile.eof())
		{
			char line[1000];
			inputFile.getline(line,1000);
			string s(line);

			switch(status)
			{

				//just skip header.
			case READING_HEADER:
				{
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

				//if reading vertex right now, then using TPS convert the vertex coordinates. 
			case READING_VERTEX:
				{
					//eat up all empty lines
					if(s.size()==0&&count==0)
					{					
					}
					else if(count<pointsNum)
					{
						count++;
						double coord[3];
						getXYZPLY(s,coord);
						Vec temp(coord[0], coord[1], coord[2]);
						points->push_back(temp);
					}
					else if(count==pointsNum)
					{
						status=READING_POLY;
					}
					break;					 
				}
			}
			//all vertex have been recorded just exit.
			if(status==READING_POLY)
			{
				break;
			}
		}
	}
}
//get XYZ coordinate from string line in PLY file.
void getXYZPLY(string s, double* coord)
{
	int start, end;
	start=0;
	end=s.find(" ");
	coord[0]=atof(s.substr(start,end-start).c_str());
	start=end+1;
	end=s.find(" ",start);
	coord[1]=atof(s.substr(start,end-start).c_str());
	start=end+1;
	end=s.find(" ",start);
	coord[2]=atof(s.substr(start,end-start).c_str());
}

void getTriangleFacesPLY(string s,unsigned int *idx)
{
	int start, end;
	start=0;
	end=s.find(" ");
	start=end+1;
	end=s.find(" ",start);
	idx[0]=atol(s.substr(start,end-start).c_str());
	start=end+1;
	end=s.find(" ",start);
	idx[1]=atol(s.substr(start,end-start).c_str());
	start=end+1;
	end=s.size();
	idx[2]=atol(s.substr(start,end-start).c_str());

}

bool writePointsToPly(mrpt::slam::CSimplePointsMap *points, LPTSTR referenceFilePath, LPTSTR outPutFilePath)
{
	cout<<"writing data to: "<<(*outPutFilePath)<<"..."<<endl;

	ifstream inputFile(referenceFilePath);
	ofstream outputFile(outPutFilePath);
	if(inputFile.is_open())
	{
		int status=READING_HEADER;
		int pointsNum=0;
		int count=0;
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
					//record points number
					if(s.find("element vertex")!=-1)
					{
						pointsNum=atol(s.substr(15,s.length()-15).c_str());
						if(pointsNum!=points->getPointsCount())
						{
						     inputFile.close();
							 outputFile.close();
							return false;
						}
						
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

						float x, y, z;
						points->getPoint(count, x, y, z);
						int end, start;
						//get the start Idx of color data if exists. 
						end=s.find(" ");
						start=end+1;
						end=s.find(" ",start);
						start=end+1;
						end=s.find(" ",start);
						start=end+1;
						outputFile<<x<<" "<<y<<" "<<z<<" "<<s.substr(start, s.size()-start)<<endl;
						count++;
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
	inputFile.close();
	outputFile.close();
	return true;
}



