#include "Common.h"

double thrustForce = 0.0;
double shoulderThrust = 4.0;
double thrustAngle = 0.0;
double thrustChangeRate = 0.012;
double thrustRotationSpeed = 3.0;
double maxThrustForce = 14.0;
double staticFrictionCoeff = 0.5;
double kineticFrictionCoeff = 0.3;
double boxStaticFrictionCoeff = 0.3;
double boxKineticFrictionCoeff = 0.2;

bool leftArrowPressed = false;
bool rightArrowPressed = false;

bool lava = false;

float g_timeScale = 0.8f;