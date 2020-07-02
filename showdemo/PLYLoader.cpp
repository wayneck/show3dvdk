
#include "PLYLoader.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
CPLYLoader::CPLYLoader()
{
    this->m_totalConnectedQuads = 0;
    this->m_totalConnectedPoints = 0;
    this->m_ModelData.iTotalConnectedTriangles = 0;
}

bool CPLYLoader::LoadModel(char * filename)
{
    printf("Loading %s...\n", filename);

    char * pch = strstr(filename, ".ply");
    if (pch != NULL)
    {
        FILE * file = fopen(filename, "r");
        if (!file)
        {
            printf("load PLY file %s failed\n", filename);
            return false;
        }
        fseek(file, 0, SEEK_END);
        long fileSize = ftell(file);
        try
        {
            mp_vertexXYZ = (float*)malloc(fileSize);
            mp_vertexRGB = (float*)malloc(fileSize);
        }
        catch (char *)
        {
            return false;
        }
        if (mp_vertexXYZ == NULL || mp_vertexRGB == NULL) return false;
        fseek(file, 0, SEEK_SET);
        if (file)
        {
            char buffer[1000];
            fgets(buffer, 300, file);
            //READ HEADER
            while (strncmp("element vertex", buffer, strlen("element vertex")) != 0)
            {
                fgets(buffer, 300, file);
            }
            strcpy(buffer, buffer + strlen("element vertex"));
            sscanf(buffer, "%i", &this->m_totalConnectedPoints);
            printf("first we get %d vertexs\n", this->m_totalConnectedPoints);
            //Find the number of vertexes
            fseek(file, 0, SEEK_SET);
            while (strncmp("element face", buffer, strlen("element face")) != 0)
            {
                fgets(buffer, 300, file);
            }
            strcpy(buffer, buffer + strlen("element face"));
            sscanf(buffer, "%i", &this->m_totalFaces);
            printf("second we get %d faces\n", this->m_totalFaces);
            //Go to end_header
            while (strncmp("end_header", buffer, strlen("end_header")) != 0)
            {
                fgets(buffer, 300, file);
            }
            //Read vertices
            int i = 0;
            for (int iterator = 0; iterator < this->m_totalConnectedPoints; iterator++)
            {
                fgets(buffer, 300, file);
                sscanf(buffer, "%f %f %f", &mp_vertexXYZ[i], &mp_vertexXYZ[i + 1], &mp_vertexXYZ[i + 2]);
                i += 3;
            }
            //Read faces
            for (int iterator = 0; iterator < this->m_totalFaces; iterator++)
            {
                fgets(buffer, 300, file);
                //Triangular patch
                if (buffer[0] == '3')
                {
                    int vertex1 = 0, vertex2 = 0, vertex3 = 0;
                    buffer[0] = ' ';
                    sscanf(buffer, "%i%i%i", &vertex1, &vertex2, &vertex3);
                    //point
                    m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex1]);
                    m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex1 + 1]);
                    m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex1 + 2]);
                    m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex2]);
                    m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex2 + 1]);
                    m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex2 + 2]);
                    m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex3]);
                    m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex3 + 1]);
                    m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex3 + 2]);
                    //color
                    m_ModelData.vecFaceTriangleColors.push_back(1.0);
                    m_ModelData.vecFaceTriangleColors.push_back(1.0);
                    m_ModelData.vecFaceTriangleColors.push_back(1.0);
                    m_ModelData.vecFaceTriangleColors.push_back(1.0);
                    m_ModelData.vecFaceTriangleColors.push_back(1.0);
                    m_ModelData.vecFaceTriangleColors.push_back(1.0);
                    m_ModelData.vecFaceTriangleColors.push_back(1.0);
                    m_ModelData.vecFaceTriangleColors.push_back(1.0);
                    m_ModelData.vecFaceTriangleColors.push_back(1.0);
                    m_ModelData.iTotalConnectedTriangles += 3;
                }
            }
            fclose(file);
            printf("%s Loaded!\n", filename);
            return true;
        }
        else
        {
            printf("File can't be opened\n");
            return false;
        }
    }
    else
    {
        printf("File does not have a ply extension.\n");
        return false;
    }
}

void CPLYLoader::Draw() //implemented in GLPainter, not called again
{
    glVertexPointer(3, GL_FLOAT, 0, m_ModelData.vecFaceTriangles.data());
    glColorPointer(3, GL_FLOAT, 0, m_ModelData.vecFaceTriangleColors.data());
    glDrawArrays(GL_TRIANGLES, 0, m_ModelData.iTotalConnectedTriangles);
}