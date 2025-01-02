#include "LanderSim.h"

LanderSim::LanderSim(const std::string& name, Lander* target) :
    BaseSimulator(name),
    m_object(target),
    prev_time(0.0),
    gravity(-2.0),
    previousPositions(nullptr)
{
} // LanderSimulator

LanderSim::~LanderSim()
{
    delete[] previousPositions;
} // LanderSim::~LanderSim

int LanderSim::init(double time) {

    m_object->getState(particles);

    if (previousPositions) {
        delete[] previousPositions;
    }
    previousPositions = new Vector[particles.size()];
    for (int i = 0; i < particles.size(); i++) {
        for (int j = 0; j < 3; j++) {
            // Initialize previous positions to current positions since we don't have dt
            previousPositions[i][j] = particles[i].position[j];
        }
    }

    m_object->setBoxes(boxes);
    m_object->getSprings(springs);
    initialSprings = springs;

    return 0;
}

void LanderSim::reset(double time) {
    prev_time = 0;
    springs.clear();
    springs = initialSprings;
}

int LanderSim::step(double time)
{
    PerformanceMetrics::startMeasurement("Total Step");

    PerformanceMetrics::startMeasurement("Time Step");
    // Calculate the time step
    double dt = (time - prev_time) * g_timeScale;
    prev_time = time;

    m_object->getState(particles);
    
    // Early return if no particles
    if (particles.empty()) {
        PerformanceMetrics::endMeasurement("Time Step");
        double totalTime = PerformanceMetrics::endMeasurement("Total Step");
        return 0;
    }
    PerformanceMetrics::endMeasurement("Time Step");

    //reset forces
    PerformanceMetrics::startMeasurement("Forces Reset");
    for (Particle& p : particles) {
        setVector(p.force, 0, 0, 0);
    }
    PerformanceMetrics::endMeasurement("Forces Reset");

    PerformanceMetrics::startMeasurement("Input Processing");
    //arrow key controls
    if (upArrowPressed){
        thrustForce += thrustChangeRate;
        thrustForce = (thrustForce > maxThrustForce ? maxThrustForce : thrustForce);
    }
    if (downArrowPressed){
        thrustForce -= thrustChangeRate;
        thrustForce = (thrustForce < 0 ? 0 : thrustForce);
    }

    if (rightArrowPressed){
        // rotateShip(-thrustRotationSpeed * dt);
        shoulderForce('r');
    }
    if (leftArrowPressed){
        // rotateShip(thrustRotationSpeed * dt);
        shoulderForce('l');
    }
    PerformanceMetrics::endMeasurement("Input Processing");

    PerformanceMetrics::startMeasurement("External Forces");
    //apply external forces
    for (int i = 0; i < particles.size(); i++) {
        if (std::find(fixedParticles.begin(), fixedParticles.end(), i) != fixedParticles.end()) {
            continue;
        }
        
        //gravity
        particles[i].force[1] = gravity * particles[i].mass;

        //apply ground forces
        if (particles[i].position[1] < groundY) {
            double depth = groundY - particles[i].position[1];
            double springForce = groundStiffness * depth;
            double dampingForce = -groundDamping * particles[i].velocity[1];

            double maxDampingForce = std::abs(springForce);
            dampingForce = std::min<double>(std::max<double>(dampingForce, -maxDampingForce), maxDampingForce);

            particles[i].force[1] += springForce + dampingForce;

            double normalForce = springForce + dampingForce;

            double horizontalVelocity = particles[i].velocity[0];
            double frictionForce;

            //apply friction
            // Inside the ground friction implementation
            if (std::abs(horizontalVelocity) < 0.1) { // Static friction case
                double appliedForce = particles[i].force[0];
                double maxStaticFriction = staticFrictionCoeff * normalForce;
                
                // Reduce static friction by 30% instead of 50%
                maxStaticFriction *= 0.7;
                
                if (std::abs(appliedForce) <= maxStaticFriction) {
                    particles[i].velocity[0] *= 0.9;
                    frictionForce = -appliedForce * 0.85; // Counter 85% of the applied force instead of 80%
                } else {
                    int forceSign = (appliedForce > 0) ? 1 : -1;
                    frictionForce = -maxStaticFriction * forceSign;
                }
            }   
            //kinetic friction
            else {
                // Get direction of current horizontal velocity
                int velocitySign = (particles[i].velocity[0] > 0 ? 1 : -1);
                double frictionMagnitude = kineticFrictionCoeff * normalForce;
                frictionForce = -frictionMagnitude * velocitySign;
            }

            particles[i].force[0] += frictionForce;
        }

        //apply roof forces
        if (particles[i].position[1] > roofY) {
            double depth = particles[i].position[1] - roofY;
            double penaltyForce = roofStiffness * depth;
            double dampingForce = roofDamping * particles[i].velocity[1];
            particles[i].force[1] -= penaltyForce + dampingForce;
        }

        //apply wall forces
        if (particles[i].position[0] < wallL) {
            double depth = wallL - particles[i].position[0];
            double penaltyForce = wallStiffness * depth;
            double dampingForce = -wallDamping * particles[i].velocity[0];
            particles[i].force[0] += penaltyForce + dampingForce;
        }

        //apply wall forces
        if (particles[i].position[0] > wallR) {
            double depth = particles[i].position[0] - wallR;
            double penaltyForce = wallStiffness * depth;
            double dampingForce = wallDamping * particles[i].velocity[0];
            particles[i].force[0] -= penaltyForce + dampingForce;
        }
    }
    PerformanceMetrics::endMeasurement("External Forces");

    //Check for collisions with boxes
    PerformanceMetrics::startMeasurement("Collision Detection");
    handleBoxCollisions();
    PerformanceMetrics::endMeasurement("Collision Detection");

    //apply thrust
    PerformanceMetrics::startMeasurement("Thrust Application");
    Vector thrustDirection;
    Vector leftSide, rightSide;
    VecSubtract(leftSide, particles[3].position, particles[1].position);
    VecSubtract(rightSide, particles[4].position, particles[2].position);

    // Average the two sides to get the central thrust direction
    thrustDirection[0] = (leftSide[0] + rightSide[0]) / 2.0;
    thrustDirection[1] = (leftSide[1] + rightSide[1]) / 2.0;
    thrustDirection[2] = (leftSide[2] + rightSide[2]) / 2.0;

    VecNormalize(thrustDirection);
    double thrustX = thrustForce * thrustDirection[0] / 2;
    double thrustY = -thrustForce * thrustDirection[1] / 2;

    particles[3].force[0] -= thrustX;
    particles[3].force[1] += thrustY;

    particles[4].force[0] -= thrustX;
    particles[4].force[1] += thrustY;
    PerformanceMetrics::endMeasurement("Thrust Application");

    //apply springs
    PerformanceMetrics::startMeasurement("Spring Forces");
    for (int i = 0; i < springs.size(); i++) {
        calculateSpringForces(i);
    }
    PerformanceMetrics::endMeasurement("Spring Forces");
    
    //integrate
    PerformanceMetrics::startMeasurement("Integration");
    for (int i = 0; i < particles.size(); i++) {
        if (std::find(fixedParticles.begin(), fixedParticles.end(), i) != fixedParticles.end()) {
            continue;
        }

        if (mode == 0) {  //forward euler
            particles[i].velocity[0] += (particles[i].force[0] / particles[i].mass) * dt;
            particles[i].velocity[1] += (particles[i].force[1] / particles[i].mass) * dt;
            particles[i].velocity[2] += (particles[i].force[2] / particles[i].mass) * dt;

            particles[i].position[0] += particles[i].velocity[0] * dt;
            particles[i].position[1] += particles[i].velocity[1] * dt;
            particles[i].position[2] += particles[i].velocity[2] * dt;
        }

        if (mode == 1) {  //symplectic euler
            particles[i].position[0] += particles[i].velocity[0] * dt;
            particles[i].position[1] += particles[i].velocity[1] * dt;
            particles[i].position[2] += particles[i].velocity[2] * dt;

            particles[i].velocity[0] += (particles[i].force[0] / particles[i].mass) * dt;
            particles[i].velocity[1] += (particles[i].force[1] / particles[i].mass) * dt;
            particles[i].velocity[2] += (particles[i].force[2] / particles[i].mass) * dt;
        }   

        if (mode == 2) {  //verlet
            Vector temp, acceleration;
            
            setVector(acceleration,
                particles[i].force[0] / particles[i].mass,
                particles[i].force[1] / particles[i].mass,
                particles[i].force[2] / particles[i].mass
            );

            VecCopy(temp, particles[i].position);

            for (int j = 0; j < 3; j++) {
                particles[i].position[j] = 2 * particles[i].position[j] - previousPositions[i][j] + acceleration[j] * dt * dt;

                particles[i].velocity[j] = (particles[i].position[j] - previousPositions[i][j]) / (2 * dt);
            }

            VecCopy(previousPositions[i], temp);
        }
    }
    PerformanceMetrics::endMeasurement("Integration");

    PerformanceMetrics::startMeasurement("State Update");
    m_object->setState(particles);
    PerformanceMetrics::endMeasurement("State Update");

    double totalTime = PerformanceMetrics::endMeasurement("Total Step");
    return 0;
} // LanderSim::step

void LanderSim::handleArrowKeyDown(int key) {
    switch (key) {
        case GLUT_KEY_RIGHT:
            // animTcl::OutputMessage("Right arrow key pressed\n");
            rightArrowPressed = true;
            break;
        case GLUT_KEY_LEFT:
            // animTcl::OutputMessage("Left arrow key pressed\n");
            leftArrowPressed = true;
            break;
        case GLUT_KEY_UP:
            // animTcl::OutputMessage("Up arrow key pressed\n");
            upArrowPressed = true;
            break;
        case GLUT_KEY_DOWN:
            // animTcl::OutputMessage("Down arrow key pressed\n");
            downArrowPressed = true;
            break;
    }
}

void LanderSim::handleArrowKeyUp(int key) {
    switch (key) {
        case GLUT_KEY_RIGHT:
            // animTcl::OutputMessage("Right arrow key released\n");
            rightArrowPressed = false;
            break;
        case GLUT_KEY_LEFT:
            // animTcl::OutputMessage("Left arrow key released\n");
            leftArrowPressed = false;
            break;
        case GLUT_KEY_UP:
            // animTcl::OutputMessage("Up arrow key released\n");
            upArrowPressed = false;
            break;
        case GLUT_KEY_DOWN:
            // animTcl::OutputMessage("Down arrow key released\n");
            downArrowPressed = false;
            break;
    }
}

void LanderSim::calculateSpringForces(int index) {  
    const Spring& spring = springs[index];
    Vector pos1, pos2, direction;
    VecCopy(pos1, particles[spring.p1].position);
    VecCopy(pos2, particles[spring.p2].position);

    VecSubtract(direction, pos2, pos1);
    double currentLength = VecLength(direction);

    VecNormalize(direction);

    Vector vel1, vel2, relativeVel;
    VecCopy(vel1, particles[spring.p1].velocity);
    VecCopy(vel2, particles[spring.p2].velocity);
    VecSubtract(relativeVel, vel2, vel1);

    double springForce = -spring.stiffness * (currentLength - spring.restLength);
    double dampingForce = -spring.damping * VecDotProd(relativeVel, direction);
    double totalForce = springForce + dampingForce;

    for (int i = 0; i < 3; i++) {
        particles[spring.p1].force[i] -= direction[i] * totalForce;
        particles[spring.p2].force[i] += direction[i] * totalForce;
    }
}

void LanderSim::calculateCenterOfMass(Vector& com) {
    double totalMass = 0;
    for (int i = 0; i < particles.size(); i++) {
        totalMass += particles[i].mass;
        VecAdd(com, com, particles[i].position);
    }
    VecScale(com, 1 / totalMass);
}

void LanderSim::rotateShip(double angle) {

    // instead of rotating the ship, apply force along the line from the nose
    // to the shoulders of the ship

    Vector com = {0, 0, 0};
    Vector comVelocity = {0, 0, 0};
    
    // Calculate center of mass and average velocity
    double totalMass = 0;
    for (const Particle& p : particles) {
        totalMass += p.mass;
        for (int i = 0; i < 3; i++) {
            com[i] += p.position[i] * p.mass;
            comVelocity[i] += p.velocity[i] * p.mass;
        }
    }

    for (int i = 0; i < 3; i++) {
        com[i] /= totalMass;
        comVelocity[i] /= totalMass;
    }

    // Rotate each particle around COM
    for (Particle& p : particles) {
        // Position rotation
        Vector relativePos;
        VecSubtract(relativePos, p.position, com);

        double x = relativePos[0];
        double y = relativePos[1];
        
        // Apply rotation matrix
        double newX = x * cos(angle) - y * sin(angle);
        double newY = x * sin(angle) + y * cos(angle);

        p.position[0] = com[0] + newX;
        p.position[1] = com[1] + newY;

        // Velocity rotation
        Vector relativeVel;
        VecSubtract(relativeVel, p.velocity, comVelocity);

        x = relativeVel[0];
        y = relativeVel[1];
        
        // Apply same rotation to velocity
        newX = x * cos(angle) - y * sin(angle);
        newY = x * sin(angle) + y * cos(angle);

        p.velocity[0] = comVelocity[0] + newX;
        p.velocity[1] = comVelocity[1] + newY;
    }
}

void LanderSim::shoulderForce(char side) {
    Vector forceDirection;
    setVector(forceDirection, 0, 0, 0);

    if (side == 'l'){
        VecSubtract(forceDirection, particles[1].position, particles[2].position);
        VecNormalize(forceDirection);

        double thrustX = shoulderThrust * forceDirection[0];
        double thrustY = shoulderThrust * forceDirection[1];

        particles[0].force[0] += thrustX;
        particles[0].force[1] += thrustY;
    }

    if (side == 'r'){
        VecSubtract(forceDirection, particles[2].position, particles[1].position);
        VecNormalize(forceDirection);

        double thrustX = shoulderThrust * forceDirection[0];
        double thrustY = shoulderThrust * forceDirection[1];

        particles[0].force[0] += thrustX;
        particles[0].force[1] += thrustY;
    }
    
}

bool LanderSim::checkCollision(Particle& p, Box& b, Vector& normal, double& depth) {
    if (p.position[0] > b.position[0] && p.position[0] < b.position[0] + b.width && 
        p.position[1] > b.position[1] && p.position[1] < b.position[1] + b.height){

        double distLeft = p.position[0] - b.position[0];
        double distRight = (b.position[0] - p.position[0]) + b.width;
        double distTop = (b.position[1] - p.position[1]) + b.height;
        double distBottom = p.position[1] - b.position[1];

        // Find closest edge
        depth = std::min<double>(std::min<double>(distLeft, distRight), std::min<double>(distTop, distBottom));

        // Set normal vector based on closest edge
        setVector(normal, 0, 0, 0);
        if (depth == distLeft) {
            normal[0] = -1;
        } else if (depth == distRight) {
            normal[0] = 1;
        } else if (depth == distTop) {
            normal[1] = 1;
        } else {  // bottom
            normal[1] = -1;
        }

        return true;
    }
    return false;
}

void LanderSim::handleBoxCollisions() {
    for (int i = 0; i < particles.size(); i++) {
        for (Box& box : boxes) {
            Vector normal;
            double depth;
            if (checkCollision(particles[i], box, normal, depth)) {
                if (lava) {
                    springs.clear();
                    m_object->setSprings(springs);
                }
                // Apply penalty force
                double penaltyForce = boxStiffness * depth;
                double dampingForce = -boxDamping * (particles[i].velocity[0] * normal[0] + 
                                                     particles[i].velocity[1] * normal[1]);
                
                double maxDampingForce = std::abs(penaltyForce);
                dampingForce = std::min<double>(std::max<double>(dampingForce, -maxDampingForce), maxDampingForce);

                double normalForce = penaltyForce + dampingForce;

                // Apply normal force
                particles[i].force[0] += normal[0] * normalForce;
                particles[i].force[1] += normal[1] * normalForce;

                // Calculate friction like ground implementation
                double horizontalVelocity = particles[i].velocity[0];
                double frictionForce;

                //apply friction
                if (std::abs(horizontalVelocity) < 0.1) { // Static friction case
                    double appliedForce = particles[i].force[0];
                    double maxStaticFriction = boxStaticFrictionCoeff * normalForce;
                    
                    // Apply the same adjustment here
                    maxStaticFriction *= 0.7;
                    
                    if (std::abs(appliedForce) <= maxStaticFriction) {
                        particles[i].velocity[0] *= 0.9;
                        frictionForce = -appliedForce * 0.85;
                    } else {
                        int forceSign = (appliedForce > 0) ? 1 : -1;
                        frictionForce = -maxStaticFriction * forceSign;
                    }
                }   
                //kinetic friction
                else {
                    // Get direction of current horizontal velocity
                    int velocitySign = (particles[i].velocity[0] > 0 ? 1 : -1);
                    double frictionMagnitude = boxKineticFrictionCoeff * normalForce;
                    frictionForce = -frictionMagnitude * velocitySign;
                }

                particles[i].force[0] += frictionForce;
            }
        }
    }
}

int LanderSim::command(int argc, myCONST_SPEC char** argv) {
    if (argc < 1) {
        animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
        return TCL_ERROR;
    }
    else if (strcmp(argv[0], "reset") == 0) {
        reset(0.0);
    }
    else if (strcmp(argv[0], "link") == 0) {
        if (argc == 3) {
            const char* sysName = argv[1];
            numSprings = atoi(argv[2]);

            BaseSystem* sys = GlobalResourceManager::use()->getSystem(sysName);
            if (sys == nullptr) {
                animTcl::OutputMessage("Could not find system named '%s'", sysName);
                return TCL_ERROR;
            }

            Lander* springSystem = dynamic_cast<Lander*>(sys);
            if (springSystem == nullptr) {
                animTcl::OutputMessage("System '%s' is not a Lander", sysName);
                return TCL_ERROR;
            }

            m_object = springSystem;

        }
        else {
            animTcl::OutputMessage("Usage: link <system_name> <num_springs>");
            return TCL_ERROR;
        }
    }

    else if (strcmp(argv[0], "spring") == 0) {
        if (argc == 6) {
            if (m_object == nullptr) {
                animTcl::OutputMessage("No system linked. Use 'link' command first");
                return TCL_ERROR;
            }

            int index1 = atoi(argv[1]);
            int index2 = atoi(argv[2]);
            double springConstant = atof(argv[3]);
            double dampingConstant = atof(argv[4]);
            double restLength = atof(argv[5]);

            m_object->getState(particles);

            if (index1 < 0 || index1 >= particles.size() || 
                index2 < 0 || index2 >= particles.size()) {
                animTcl::OutputMessage("Invalid particle indices. Must be between 0 and %d", 
                    particles.size() - 1);
                return TCL_ERROR;
            }
            if (springConstant <= 0) {
                animTcl::OutputMessage("Spring constant must be positive");
                return TCL_ERROR;
            }
            if (restLength < 0) {
                Vector pos1, pos2, result; 
                VecCopy(pos1, particles[index1].position);
                VecCopy(pos2, particles[index2].position);
                VecSubtract(result, pos2, pos1);
                restLength = VecLength(result);
            }

            springs.push_back(Spring{index1, index2, restLength, springConstant, dampingConstant});

            m_object->addSpring(index1, index2, restLength, springConstant, dampingConstant);
        }
        else {
            animTcl::OutputMessage("Usage: spring <index1> <index2> <ks> <kd> <restLength>");
            return TCL_ERROR;
        }
    }

    else if (strcmp(argv[0], "fix") == 0) {
        if (argc == 2) {
            int index = atoi(argv[1]);
            fixedParticles.push_back(index);
        }
        else {
            animTcl::OutputMessage("Usage: fix <index>");
            return TCL_ERROR;
        }
    }

    else if (strcmp(argv[0], "integration") == 0) {
        if (argc == 3) {
            if (strcmp(argv[1], "euler") == 0) {
                mode = 0;
            }
            else if (strcmp(argv[1], "symplectic") == 0) {
                mode = 1;
            }
            else if (strcmp(argv[1], "verlet") == 0) {
                mode = 2;
            }
            else {
                animTcl::OutputMessage("Invalid integration mode. Must be euler, symplectic, or verlet");
                return TCL_ERROR;
            }
             double newDt = atof(argv[2]);
        }
        else {
            animTcl::OutputMessage("Usage: integration <mode> <dt>");
            return TCL_ERROR;
        }
    }

    else if (strcmp(argv[0], "ground") == 0) {
        if (argc == 3) {
            groundStiffness = atof(argv[1]);
            groundDamping = atof(argv[2]);
        }
        else {
            animTcl::OutputMessage("Usage: ground <ks> <kd>");
            return TCL_ERROR;
        }
    }

    else if (strcmp(argv[0], "gravity") == 0) {
        if (argc == 2) {
            gravity = atof(argv[1]);
        }
        else {
            animTcl::OutputMessage("Usage: gravity <g>");
            return TCL_ERROR;
        }
    }

    else if (strcmp(argv[0], "drag") == 0) {
    if (argc == 2) {
        double kdrag = atof(argv[1]);
        if (kdrag < 0) {
            animTcl::OutputMessage("Drag coefficient must be positive");
            return TCL_ERROR;
        }
        dragCoefficient = kdrag;
    }
    else {
            animTcl::OutputMessage("Usage: drag <kdrag>");
            return TCL_ERROR;
        }
    }

    else if (strcmp(argv[0], "benchmark") == 0) {
        if (argc == 2) {
            int frames = atoi(argv[1]);
            
            animTcl::OutputMessage("\nStarting benchmark for %d frames...", frames);
            PerformanceMetrics::resetMetrics();
            
            // Run simulation for specified frames
            for(int i = 0; i < frames; i++) {
                step(i * 0.016667); // 60fps timestep
            }
            
            PerformanceMetrics::logResults();
            return TCL_OK;
        }
        animTcl::OutputMessage("Usage: benchmark <number_of_frames>");
        return TCL_ERROR;
    }

    glutPostRedisplay();
    return TCL_OK;
}