#ifndef MY_BEZIER_H
#define MY_BEZIER_H

#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>

#include "shared/opengl.h"

class Bezier : public BaseSystem
{

public:
	Bezier(const std::string& name);
	virtual void getState(double* p);
	virtual void setState(double* p);
	void reset(double time);

	void displayControlPoint(Vector p, float r);
	void display(GLenum mode = GL_RENDER);

	int command(int argc, myCONST_SPEC char** argv);

protected:
	Vector p0, p1, p2, p3;
};
#endif
