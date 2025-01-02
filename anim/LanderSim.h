#pragma once
#ifndef SPRING_MASS_SIMULATOR_H
#define SPRING_MASS_SIMULATOR_H

#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "Lander.h"
#include "GlobalResourceManager.h"

#include <string>
#include <vector>
#include <algorithm>
#include "Common.h"
#include "PerformanceMetrics.h"
// A simulator for the spring-mass system
class LanderSim : public BaseSimulator
{
public:
    LanderSim(const std::string& name, Lander* target);
    ~LanderSim();

    void reset(double time);
    int init(double time);
    int step(double time);
    void handleArrowKeyDown(int key);
    void handleArrowKeyUp(int key);
    void calculateSpringForces(int index);
    void calculateCenterOfMass(Vector& com);
    void rotateShip(double angle);
    void shoulderForce(char side);
    bool checkCollision(Particle& p, Box& b, Vector& normal, double& distance);
    void handleBoxCollisions();

    int command(int argc, myCONST_SPEC char** argv);

protected:
    Lander* m_object; 
    double prev_time;
    double gravity;
    int numSprings;
    int mode = 1; //0 = forward euler, 1 = symplectic euler, 2 = verlet

    const double groundY = -5.0;
    const double wallR = 5.0;
    const double wallL = -5.0;
    const double roofY = 5.0;
    double groundStiffness = 50000.0;
    double groundDamping = 500.0;
    double dragCoefficient = 0.0;
    double wallStiffness = 10000.0;
    double wallDamping = 70.0;
    double roofStiffness = 10000.0;
    double roofDamping = 70.0;

    double boxStiffness = 10000.0;
    double boxDamping = 70.0;

    bool prevLeftArrow = false;
    bool prevRightArrow = false;
    bool upArrowPressed = false;
    bool downArrowPressed = false;

    std::vector<int> fixedParticles;
    std::vector<Particle> particles;
    std::vector<Spring> springs;
    std::vector<Spring> initialSprings;
    std::vector<Box> boxes;
    Vector* previousPositions;
};

#endif