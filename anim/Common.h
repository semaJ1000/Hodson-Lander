#pragma once
#ifndef COMMON_H
#define COMMON_H

#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <vector>

// A class representing a particle in the spring-mass system
struct Particle {
    Vector position;   // Position of the particle
    Vector velocity;   // Velocity of the particle
    Vector force;      // Accumulated force on the particle
    double mass;       // Mass of the particle
};

// A class representing a spring connecting two particles
struct Spring {
    int p1;     // Index of the first particle
    int p2;     // Index of the second particle
    double restLength; // Rest length of the spring
    double stiffness;  // Spring constant
    double damping;    // Damping constant
};

struct Box {
    Vector position;
    double width;
    double height;
};

// extern double dt;
// extern double newDt;
extern double thrustForce;
extern double shoulderThrust;
extern double maxThrustForce;
extern double thrustAngle;
extern double thrustChangeRate;
extern double thrustRotationSpeed;
extern double staticFrictionCoeff;
extern double kineticFrictionCoeff;
extern double boxStaticFrictionCoeff;
extern double boxKineticFrictionCoeff;

extern bool leftArrowPressed;
extern bool rightArrowPressed;

extern bool lava;

extern float g_timeScale;

#endif