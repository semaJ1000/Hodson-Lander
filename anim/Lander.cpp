#include "Lander.h"

Lander::Lander(const std::string& name) : 
    BaseSystem(name),
    m_sx(0.5f),
	m_sy(0.5f),
	m_sz(0.5f)
{
	// Clear any existing particles
    particles.clear();
    initialParticles.clear();
	boxes.clear();

	initializeLander();
	// simpleParticles();
	sparseBlockLayout();
	// manyBlockLayout();
	// longBoxLayout();

	initialSprings = springs;
	initialParticles = particles;

}

void Lander::initializeLander() {
	particles.clear();
	initialParticles.clear();
	springs.clear();
	initialSprings.clear();
    // Starting position and velocity for the lander
    startingPosition, startingVelocity;
	setVector(startingPosition, 1, 2.0, 0); // Can be modified to change starting position
	setVector(startingVelocity, 0, 0, 0);       // Can be modified to change starting velocity
    
    // Create particles for spaceship shape
    double weight = 1.0;
    
    // Nose (front) - offset from starting position
    Particle nose;
    setVector(nose.position, 
        startingPosition[0] + 0.3,  // Centered relative to body 
        startingPosition[1],        // At the top
        startingPosition[2]);
    setVector(nose.velocity, startingVelocity[0], startingVelocity[1], startingVelocity[2]);
    setVector(nose.force, 0, 0, 0);
    nose.mass = weight;
    
    // Body points (in a square formation)
    Particle topLeft, topRight, bottomLeft, bottomRight;
    
    // Position body points relative to starting position
    setVector(topLeft.position, 
        startingPosition[0],        // Left side
        startingPosition[1] - 0.5,  // Below nose
        startingPosition[2]);
    setVector(topRight.position,
        startingPosition[0] + 0.6,  // Right side 
        startingPosition[1] - 0.5,  // Below nose
        startingPosition[2]);
    setVector(bottomLeft.position,
        startingPosition[0],        // Left side
        startingPosition[1] - 1.0,  // Bottom
        startingPosition[2]);
    setVector(bottomRight.position,
        startingPosition[0] + 0.6,  // Right side
        startingPosition[1] - 1.0,  // Bottom
        startingPosition[2]);
    
    // Set properties for all body points
    for (Particle* p : {&topLeft, &topRight, &bottomLeft, &bottomRight}) {
        setVector(p->velocity, startingVelocity[0], startingVelocity[1], startingVelocity[2]);
        setVector(p->force, 0, 0, 0);
        p->mass = weight;
    }

    // Add particles in order
    particles = {nose, topLeft, topRight, bottomLeft, bottomRight};
    initialParticles = particles;  // Save initial state

    // Add structural springs
    // Nose connections
    int stiffness = 3000.0;
    int damping = 200.0;
    addSpring(0, 1, stiffness, damping, distance(particles[0], particles[1])); // nose to topLeft
    addSpring(0, 2, stiffness, damping, distance(particles[0], particles[2])); // nose to topRight
    
    // Body frame
    addSpring(1, 2, stiffness, damping, distance(particles[1], particles[2])); // top
    addSpring(3, 4, stiffness, damping, distance(particles[3], particles[4])); // bottom
    addSpring(1, 3, stiffness, damping, distance(particles[1], particles[3])); // left
    addSpring(2, 4, stiffness, damping, distance(particles[2], particles[4])); // right
    
    // Cross bracing for stability
    addSpring(1, 4, stiffness, damping, distance(particles[1], particles[4])); // diagonal
    addSpring(2, 3, stiffness, damping, distance(particles[2], particles[3])); // diagonal
}

void Lander::simpleParticles() {
	particles.clear();
	initialParticles.clear();
	for (int i = 0; i < 50; i++) {
		Particle p;
		// Random position between -5 and 5 for x and y
		double x = (rand() % 1001 - 500) / 100.0; // -5 to 5 with better precision
		double y = (rand() % 1001 - 500) / 100.0; // -5 to 5 with better precision
		
		// Clamp positions to +-5
		x = std::min<double>(5.0, std::max<double>(-5.0, x));
		y = std::min<double>(5.0, std::max<double>(-5.0, y));
		
		setVector(p.position, x, y, 0);
		
		// Random velocity between -5 and 5 for x and y
		setVector(p.velocity,
			(rand() % 100 - 50) / 10.0,  // -5 to 5
			(rand() % 100 - 50) / 10.0,  // -5 to 5
			0);
		setVector(p.force, 0, 0, 0);
		p.mass = 1.0;
		particles.push_back(p);
	}
}

void Lander::sparseBlockLayout() {
	//create obstacles
	boxes.clear();
	Box box1;
    setVector(box1.position, -3, 0, 0);
    box1.width = 3.0;
    box1.height = 0.5;
    boxes.push_back(box1);

	Box box2;
    setVector(box2.position, 3, 0, 0);
    box2.width = 0.5;
    box2.height = 3.5;
    boxes.push_back(box2);

	Box box3;
    setVector(box3.position, -3, -5, 0);
    box3.width = 2.0;
    box3.height = 2.0;
    boxes.push_back(box3);
}

void Lander::longBoxLayout() {
	Box box1;
	setVector(box1.position, -5, -5, 0);
	box1.width = 10.0;
	box1.height = 0.5;
	boxes.push_back(box1);
}

void Lander::manyBlockLayout() {
    boxes.clear();
    
    for (int i = 0; i < 1000; i++) {
        Box box;
        // Random position between -5 and 5
        double x = (rand() % 1001 - 500) / 100.0;
        double y = (rand() % 1001 - 500) / 100.0;
        
        setVector(box.position, x, y, 0);
        
        // Random size between 0.2 and 1.0
        box.width = (rand() % 81 + 20) / 100.0;
        box.height = (rand() % 81 + 20) / 100.0;
        
        boxes.push_back(box);
    }
}

void Lander::getState(std::vector<Particle>& p)
{
    p.clear();
    for (int i = 0; i < particles.size(); i++) {
        p.push_back(particles[i]); // Copy entire particle
    }
}

void Lander::getSprings(std::vector<Spring>& s)
{
    s.clear();
    for (int i = 0; i < springs.size(); i++) {
        s.push_back(springs[i]); // Copy entire spring
    }
}

void Lander::setSprings(std::vector<Spring>& s)
{
    springs.clear();
    for (int i = 0; i < s.size(); i++) {
        springs.push_back(s[i]); // Copy entire spring
    }
}

void Lander::setBoxes(std::vector<Box>& b)
{
    b.clear();
    for (int i = 0; i < boxes.size(); i++) {
        b.push_back(boxes[i]); // Copy entire box
    }
}

void Lander::setState(std::vector<Particle>& p)
{
    for (int i = 0; i < particles.size(); i++) {
        particles[i] = p[i]; // Copy entire particle
    }
}

void Lander::reset(double time)
{
    setState(initialParticles);
	springs.clear();
	springs = initialSprings;
}

void Lander::addSpring(int a, int b, double stiffness, double damping, double restLength) {
    Spring spring;
    spring.p1 = a;
    spring.p2 = b;
    spring.stiffness = stiffness;
    spring.damping = damping;
    spring.restLength = restLength;
    springs.push_back(spring);
}

int Lander::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "read") == 0)
	{
		if (argc == 2)
		{
			m_model.ReadOBJ(argv[1]);
			glmFacetNormals(&m_model);
			glmVertexNormals(&m_model, 90);
			return TCL_OK;
		}
		else
		{
			animTcl::OutputMessage("Usage: read <file_name>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "scale") == 0)
	{
		if (argc == 4)
		{
			m_sx = (float)atof(argv[1]);
			m_sy = (float)atof(argv[2]);
			m_sz = (float)atof(argv[3]);
		}
		else
		{
			animTcl::OutputMessage("Usage: scale <sx> <sy> <sz> ");
			return TCL_ERROR;

		}
	}
	else if (strcmp(argv[0], "pos") == 0)
	{
		if (argc == 4)
		{
			m_pos[0] = atof(argv[1]);
			m_pos[1] = atof(argv[2]);
			m_pos[2] = atof(argv[3]);
		}
		else
		{
			animTcl::OutputMessage("Usage: pos <x> <y> <z> ");
			return TCL_ERROR;

		}
	}
	else if (strcmp(argv[0], "reset") == 0)
	{
		reset(0.0);
	}
	else if (strcmp(argv[0], "dim") == 0){
		if (argc == 2){
			int numParticles = atoi(argv[1]);
			particles.clear();
			initialParticles.clear();
			
			for(int i = 0; i < numParticles; i++) {
				Particle p;
				setVector(p.position, 0, 0, 0);
				setVector(p.velocity, 0, 0, 0);
				setVector(p.force, 0, 0, 0);
				p.mass = 1.0;
				particles.push_back(p);
				initialParticles.push_back(p);
			}
		} else {
			animTcl::OutputMessage("Usage: dim <num_particles>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "particle") == 0){
		if (argc == 9){
			int index = atoi(argv[1]);
			if (index >= 0 && index < particles.size()) {
				particles[index].mass = atof(argv[2]);
				particles[index].position[0] = atof(argv[3]); 
				particles[index].position[1] = atof(argv[4]);
				particles[index].position[2] = atof(argv[5]);
				particles[index].velocity[0] = atof(argv[6]);
				particles[index].velocity[1] = atof(argv[7]); 
				particles[index].velocity[2] = atof(argv[8]);

				initialParticles[index].mass = atof(argv[2]);
				initialParticles[index].position[0] = atof(argv[3]);
				initialParticles[index].position[1] = atof(argv[4]); 
				initialParticles[index].position[2] = atof(argv[5]);
				initialParticles[index].velocity[0] = atof(argv[6]);
				initialParticles[index].velocity[1] = atof(argv[7]);
				initialParticles[index].velocity[2] = atof(argv[8]);
			} else {
				animTcl::OutputMessage("Invalid particle index");
				return TCL_ERROR;
			}
		} else {
			animTcl::OutputMessage("Usage: particle <index> <mass> <x y z vx vy vz>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "all_velocities") == 0) {
		if (argc == 4) {
			double vx = atof(argv[1]);
			double vy = atof(argv[2]);
			double vz = atof(argv[3]);

			for (int i = 0; i < particles.size(); i++) {
				particles[i].velocity[0] = vx;
				particles[i].velocity[1] = vy;
				particles[i].velocity[2] = vz;

				initialParticles[i].velocity[0] = vx;
				initialParticles[i].velocity[1] = vy;
				initialParticles[i].velocity[2] = vz;
			}
		} else {
			animTcl::OutputMessage("Usage: all_velocities <vx vy vz>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "sparseBlockLayout") == 0) {
		sparseBlockLayout();
	}
	else if (strcmp(argv[0], "manyBlockLayout") == 0) {
		manyBlockLayout();
	}

	glutPostRedisplay();
	return TCL_OK;

}	// Lander::command

void Lander::drawBoxes() {
    // Save current OpenGL state
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glMatrixMode(GL_MODELVIEW);
    
    for (int i = 0; i < boxes.size(); i++) {
        // Draw corner spheres
        // glPushMatrix();
        // set_colour(1.0, 0, 0);
        // glTranslated(boxes[i].position[0], boxes[i].position[1], boxes[i].position[2]);
        // glutSolidSphere(0.05, 20, 20);
        // glTranslated(boxes[i].width, 0, 0);
        // glutSolidSphere(0.05, 20, 20);
        // glTranslated(0, boxes[i].height, 0);
        // glutSolidSphere(0.05, 20, 20);
        // glTranslated(-boxes[i].width, 0, 0);
        // glutSolidSphere(0.05, 20, 20);
        // glPopMatrix();

        // Draw connecting lines
        glDisable(GL_LIGHTING);
        glColor3f(1.0, 0.0, 0.0); // Red color
        glBegin(GL_QUADS); // Use GL_QUADS instead of GL_LINE_LOOP for solid fill
        glVertex3d(boxes[i].position[0], boxes[i].position[1], boxes[i].position[2]);
        glVertex3d(boxes[i].position[0] + boxes[i].width, boxes[i].position[1], boxes[i].position[2]);
        glVertex3d(boxes[i].position[0] + boxes[i].width, boxes[i].position[1] + boxes[i].height, boxes[i].position[2]);
        glVertex3d(boxes[i].position[0], boxes[i].position[1] + boxes[i].height, boxes[i].position[2]);
        glEnd();
    }

    // Restore OpenGL state
    glPopAttrib();
}

void Lander::display(GLenum mode)
{
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);  // White background
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glEnable(GL_LIGHTING);
    glMatrixMode(GL_MODELVIEW);
    glPushAttrib(GL_ALL_ATTRIB_BITS);
	
	drawBoxes();

	// Draw Spaceship
	for (int i = 0; i < particles.size(); i++) {
		glPushMatrix();

		glTranslated(particles[i].position[0], particles[i].position[1], particles[i].position[2]);
		set_colour(0.3, 0.0, 0.4); // Dark purple for ship
		glutSolidSphere(0.05, 20, 20);

		glPopMatrix();
	}

	glDisable(GL_LIGHTING);
	glColor3f(0.0f, 0.0f, 0.0f); // Black color for lines
	glBegin(GL_LINES);
	for (const Spring& spring : springs) {
		glVertex3f(particles[spring.p1].position[0], 
				   particles[spring.p1].position[1],
				   particles[spring.p1].position[2]);
		glVertex3f(particles[spring.p2].position[0], 
				   particles[spring.p2].position[1], 
				   particles[spring.p2].position[2]);
	}

	// Only draw thrust vectors if we have enough particles
	if (thrustForce != 0 && particles.size() >= 5) {
		glColor3f(0.0f, 0.0f, 0.0f); // Black color for thrust
		
		// Calculate thrust direction from ship orientation
		Vector thrustDirection;
		VecSubtract(thrustDirection, particles[3].position, particles[1].position);
		VecNormalize(thrustDirection);
		
		// Draw thrust vectors from spaceship point
		glVertex3f(particles[3].position[0],
				   particles[3].position[1],
				   particles[3].position[2]);
		glVertex3f(particles[3].position[0] + thrustDirection[0] * (thrustForce / 1.5) * 0.05,
				   particles[3].position[1] + thrustDirection[1] * (thrustForce / 1.5) * 0.05,
				   particles[3].position[2]);
				   
		glVertex3f(particles[4].position[0],
				   particles[4].position[1],
				   particles[4].position[2]);
		glVertex3f(particles[4].position[0] + thrustDirection[0] * (thrustForce / 1.5) * 0.05,
				   particles[4].position[1] + thrustDirection[1] * (thrustForce / 1.5) * 0.05,
				   particles[4].position[2]);
	}

	// Draw shoulder thrust vectors
	if (leftArrowPressed && particles.size() >= 3) {
		glColor3f(0.0f, 0.0f, 0.0f); // Black for left shoulder thrust
		Vector forceDirection;
		VecSubtract(forceDirection, particles[2].position, particles[1].position);
		VecNormalize(forceDirection);
		
		glVertex3f(particles[0].position[0],
				   particles[0].position[1],
				   particles[0].position[2]);
		glVertex3f(particles[0].position[0] + forceDirection[0] * shoulderThrust * 0.075,
				   particles[0].position[1] + forceDirection[1] * shoulderThrust * 0.075,
				   particles[0].position[2]);
	}

	if (rightArrowPressed) {
		glColor3f(0.0f, 0.0f, 0.0f); // Black for right shoulder thrust
		Vector forceDirection;
		VecSubtract(forceDirection, particles[1].position, particles[2].position);
		VecNormalize(forceDirection);
		
		glVertex3f(particles[0].position[0],
				   particles[0].position[1],
				   particles[0].position[2]);
		glVertex3f(particles[0].position[0] + forceDirection[0] * shoulderThrust * 0.075,
				   particles[0].position[1] + forceDirection[1] * shoulderThrust * 0.075,
				   particles[0].position[2]);
	}

	// Draw boundaries
	glVertex3f(5, 5, 0);
	glVertex3f(5, -5, 0);
	glVertex3f(-5, 5, 0); 
	glVertex3f(-5, -5, 0);
	glVertex3f(5, 5, 0);
	glVertex3f(-5, 5, 0);
	glVertex3f(5, -5, 0);
	glVertex3f(-5, -5, 0);
	glEnd();
	glEnable(GL_LIGHTING);

	glPopAttrib();
}	// Lander::display