#ifndef PLYREADER_H_
#define PLYREADER_H_

#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>
using namespace std;

struct SModelData
{
    vector <float> vecFaceTriangles; // = face * 9
    vector <float> vecFaceTriangleColors; // = face * 9
    int iTotalConnectedTriangles;
};

class CPLYLoader
{
public:
    CPLYLoader();
    bool LoadModel(char * filename);
    void Draw();
private:
    float * mp_vertexXYZ;
    float * mp_vertexRGB;
    int m_totalConnectedQuads;
    int m_totalConnectedPoints;
    int m_totalFaces;
    SModelData m_ModelData;
};

#endif