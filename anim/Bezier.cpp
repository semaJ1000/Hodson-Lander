#include <math.h>
#include <vector>

#include "Bezier.h"

Bezier::Bezier(const std::string& name) : BaseSystem(name) {
    // Set the control points
    setVector(p0, -5, -5, 0);
    setVector(p1, -4, 5, 0);
    setVector(p2, 3, -5, 5);
    setVector(p3, 5, 3, 1);

    // Generate the samples of the curve uniformly in t
    // Do this the direct way on the polynomial 
    //setSamplePointsDirect();

    // Do this the recursive way using de Casteljau
    // setSamplePointsdeCasteljau();
}


void Bezier::getState(double* p) {

}


void Bezier::setState(double* p) {

}


void Bezier::reset(double time) {

}

int Bezier::command(int argc, myCONST_SPEC char** argv) {
    glutPostRedisplay();
    return TCL_OK;
}

void Bezier::displayControlPoint(Vector p, float r) {
    glPointSize(r);
    glBegin(GL_POINTS);
    glVertex3dv(p);
    glEnd();
}

void Bezier::display(GLenum mode) {
    glEnable(GL_LIGHTING);
    glMatrixMode(GL_MODELVIEW);
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_COLOR_MATERIAL);

    // Draw the control points
    glColor3f(1, 0, 0);
    displayControlPoint(p0, 10.0);
    displayControlPoint(p1, 10.0);
    displayControlPoint(p2, 10.0);
    displayControlPoint(p3, 10.0);

    // Draw the intermediate samples along the curve
    // glColor3f(0, 0, 0);
    // displaySamplePoints(3.0);

    // Draw the lines in between the samples
    // glColor3f(0.3, 0.7, 0.1);
    // displaySampledCurve(1.5);

    glPopAttrib();
}
