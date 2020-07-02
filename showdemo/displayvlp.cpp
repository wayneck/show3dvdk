
#include "PLYLoader.h"
#include <GL/glut.h>
#include <math.h>

CPLYLoader plyLoader;

void init()
{
    glClearColor(0, 0, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnable(GL_COLOR_ARRAY);
    plyLoader.LoadModel("p1.ply");
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT);
    plyLoader.Draw();
    glFlush();
}

int main(int argc, char* argv[])
{
    glutInit(&argc, (char **)argv);
    glutInitWindowSize(500, 500);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("load model");
    init();
    glutDisplayFunc(display);
    glutMainLoop();
    return 0;
}