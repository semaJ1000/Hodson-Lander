#pragma once
#ifndef SPRING_MASS_SYSTEM_H
#define SPRING_MASS_SYSTEM_H

#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "shared/opengl.h"
#include "BaseSystem.h"

#include <string>
#include <vector>
#include "Common.h"

class Lander : public BaseSystem
{
public:
    Lander(const std::string& name);
    void initializeLander();
    void simpleParticles();
    void manyBlockLayout();
    void sparseBlockLayout();
    void longBoxLayout();
    virtual void getState(std::vector<Particle>& p);
    virtual void getSprings(std::vector<Spring>& s);
    virtual void setSprings(std::vector<Spring>& s);
    virtual void setBoxes(std::vector<Box>& b);
    virtual void setState(std::vector<Particle>& p);
    void reset(double time);
    void addSpring(int a, int b, double stiffness, double damping, double restLength);

    int command(int argc, myCONST_SPEC char** argv);

    void drawBoxes();
    void display(GLenum mode = GL_RENDER);

protected:

    double distance(const Particle& p1, const Particle& p2) {
        double dx = p1.position[0] - p2.position[0];
        double dy = p1.position[1] - p2.position[1];
        double dz = p1.position[2] - p2.position[2];
        return sqrt(dx*dx + dy*dy + dz*dz);
    }

    float m_sx;
	float m_sy;
	float m_sz;

	Vector m_pos;
    Vector startingPosition;
    Vector startingVelocity;
    std::vector<Particle> particles;
    std::vector<Particle> initialParticles;
    std::vector<Spring> springs;
    std::vector<Spring> initialSprings;
    std::vector<Box> boxes;

    GLMmodel m_model;
};

#endif


