#include "MassSpringSystemSimulator.h"




struct Masspoint {
	Vec3 position;
	Vec3 velocity;
	bool isFixed;
	bool isEmpty = true;
	Vec3 springForce;
	Vec3 tempPosition;
	Vec3 tempVelocity;
};

struct Spring {
	int masspoint1;
	int masspoint2;
	float initialLength;
	float currentLength;
	bool isEmpty = true;
};


Masspoint * masspoints;
Spring * springs;
int masspointCount, springCount, massLength, springLength;


MassSpringSystemSimulator::MassSpringSystemSimulator() 
{
	m_fMass = 10;
	m_fStiffness = 40;
	m_fDamping = 0;
	m_iIntegrator = 0;
	masspointCount = 0;
	springCount = 0;
	masspoints = new Masspoint[100];
	springs = new Spring[100];

	


}

const char * MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo 1, Demo 2, Demo 3 , Demo 4";
}

const char * MassSpringSystemSimulator::getIntegrationMethod() {
	return "Euler, Midpoint, LeapFrog";
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	m_fMass = 10;
	m_fStiffness = 40;
	m_fDamping = 0;
	
	
	//masspoints = new Masspoint[100];
	//springs = new Spring[100];

	for (int i = 0; i < masspointCount; i++) {
		masspoints[i].isEmpty = true;
	}
	for (int i = 0; i < springCount; i++) {
		springs[i].isEmpty = true;
	}


	masspointCount = 0;
	springCount = 0;
	cout << "Reset";

}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestScene)
	{
	case 0:break;
	;
	case 1:break;
	case 2:break;
	default:break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	//TODO: copy from SimulatorTemplate and implement for the likes of gravity etc via going through masspoint array and adding external forces onto springForce
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		m_externalForce += inputWorld;
	}
	else {
		
	}
	return;
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestScene = testCase;
	
	switch (m_iTestScene)
	{
	case 0:
		cout << "Demo 1 !\n";
		setUpDemo1();
		break;
	case 1:
		cout << "Demo 2!\n";
		setUpDemo2();
		break;
	case 2:
		cout << "Demo 3 !\n";
		setUpDemo3();
		break;
	case 3: 
		cout << "Demo 4 !\n";
		setUpDemo4();
		break;
	default:
		cout << "Empty Demo!\n";
		break;
	}
}

void MassSpringSystemSimulator::notifyMethodChanged(int integrationMethod) {
	m_iIntegrator = integrationMethod;
	switch (m_iIntegrator)
	{
	case 0:
		cout << "Euler !\n";
		break;
	case 1:
		cout << "Midpoint!\n";
		break;
	case 2:
		cout << "LeapFrog!\n";
		break;
	
	default:
		cout << "Empty Method!\n";
		break;
	}
}



void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	
	if (masspoints[masspointCount].isEmpty) {
		masspoints[masspointCount].position = position;
		masspoints[masspointCount].velocity = Velocity;
		masspoints[masspointCount].isFixed = isFixed;
		masspoints[masspointCount].isEmpty = false;
		masspointCount++;
		return masspointCount - 1;
	}
	return -1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	if (springs[springCount].isEmpty) {
		springs[springCount].masspoint1= masspoint1;
		springs[springCount].masspoint2 = masspoint2;
		springs[springCount].initialLength = initialLength;
		springs[springCount].isEmpty = false;
		springCount++;
	}
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return masspointCount;
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return springCount;
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return masspoints[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return masspoints[index].velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	m_externalForce = force;
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	
	switch (m_iIntegrator) {
	case EULER: 
		
		integrateEuler(timeStep);
		break;
	case MIDPOINT:
		integrateMidpoint(timeStep);
		break;
	case LEAPFROG:
		integrateLeapFrog(timeStep);
		break;
	default:
		break;
	}
}


void MassSpringSystemSimulator::integrateEuler(float timeStep) {
	for (int i = 0; i < masspointCount; i++) {
		masspoints[i].springForce = Vec3(0,0,0);
	}
	for (int i = 0; i < springCount; i++) {

		//Compute the float length difference between point 1 and 2 by subtracting the initial length from the current length
		float lengthDifference = norm(masspoints[springs[i].masspoint1].position - masspoints[springs[i].masspoint2].position) - springs[i].initialLength; 

		//get the normalized length vectors
		Vec3 lengthVectorAtoB = getNormalized( masspoints[springs[i].masspoint1].position - masspoints[springs[i].masspoint2].position);
		Vec3 lengthVectorBtoA = getNormalized(masspoints[springs[i].masspoint2].position - masspoints[springs[i].masspoint1].position);

		//get the directed forces from A to B and from B to A
		Vec3 forceAtoB = (-1 * m_fStiffness*lengthDifference)*lengthVectorAtoB;
		Vec3 forceBtoA = (-1 * m_fStiffness*lengthDifference)*lengthVectorBtoA;

		//write the directed forces into the masspoint struct
		masspoints[springs[i].masspoint1].springForce = forceAtoB;
		masspoints[springs[i].masspoint2].springForce = forceBtoA;
		
		//std::cout << "Force "<<i<<": "<<forceAtoB.toString()<< ") \n";
		//std::cout << "Force " << i << ": "<<forceBtoA.toString() << ") \n";
	}
	for (int i = 0; i < masspointCount; i++) {
		if (!masspoints[i].isFixed) {
			masspoints[i].position = masspoints[i].position + masspoints[i].velocity*timeStep;
			if (m_iTestScene == 3) {//Collision
				if (masspoints[i].position.y < -1) {
					masspoints[i].position.y = -1;
				}
			}

			masspoints[i].velocity = masspoints[i].velocity + masspoints[i].springForce / m_fMass * timeStep;
		//	std::cout << "Point " << i << " Pos: (" << masspoints[i].position.x << ", " << masspoints[i].position.y << ", " << masspoints[i].position.z << ") \n";
			//std::cout << "Point " << i << " Vel: (" << masspoints[i].velocity.x << ", " << masspoints[i].velocity.y << ", " << masspoints[i].velocity.z << ") \n";
		}
		
	}
}







void  MassSpringSystemSimulator::integrateMidpoint(float timeStep) {
	//calculating temporary position
	for (int i = 0; i < masspointCount; i++) {
		if (!masspoints[i].isFixed) {
			masspoints[i].springForce = Vec3(0, 0, 0);
			masspoints[i].tempPosition = masspoints[i].position + timeStep / 2 * masspoints[i].velocity;
		}
	}

	//Calculating forces for temporary velocity
	for (int i = 0; i < springCount; i++) {
		//Compute the float length difference between point 1 and 2 by subtracting the initial length from the current length
		float lengthDifference = norm(masspoints[springs[i].masspoint1].position - masspoints[springs[i].masspoint2].position) - springs[i].initialLength;

		//get the normalized length vectors
		Vec3 lengthVectorAtoB = getNormalized(masspoints[springs[i].masspoint1].position - masspoints[springs[i].masspoint2].position);
		Vec3 lengthVectorBtoA = getNormalized(masspoints[springs[i].masspoint2].position - masspoints[springs[i].masspoint1].position);

		//get the directed forces from A to B and from B to A
		Vec3 forceAtoB = (-1 * m_fStiffness*lengthDifference)*lengthVectorAtoB;
		Vec3 forceBtoA = (-1 * m_fStiffness*lengthDifference)*lengthVectorBtoA;

		//write the directed forces into the masspoint struct
		masspoints[springs[i].masspoint1].springForce = forceAtoB;
		masspoints[springs[i].masspoint2].springForce = forceBtoA;
	}
	
	//Calculating final position and temporary velocity
	for (int i = 0; i < masspointCount; i++) {
		if (!masspoints[i].isFixed) {
			masspoints[i].tempVelocity = masspoints[i].velocity + timeStep / 2 * masspoints[i].springForce / m_fMass;
			masspoints[i].position = masspoints[i].position + timeStep * masspoints[i].tempVelocity;
			if (m_iTestScene == 3) {//Collision
				if (masspoints[i].position.y < -1) {
					masspoints[i].position.y = -1;
				}
			}
		}

	}
	//Calculating forces for final velocity
	for (int i = 0; i < springCount; i++) {
		//Compute the float length difference between point 1 and 2 by subtracting the initial length from the current length
		float lengthDifference = norm(masspoints[springs[i].masspoint1].tempPosition - masspoints[springs[i].masspoint2].tempPosition) - springs[i].initialLength;

		//get the normalized length vectors dependent on temporary positions 
		Vec3 lengthVectorAtoB = getNormalized(masspoints[springs[i].masspoint1].tempPosition - masspoints[springs[i].masspoint2].tempPosition);
		Vec3 lengthVectorBtoA = getNormalized(masspoints[springs[i].masspoint2].tempPosition - masspoints[springs[i].masspoint1].tempPosition);

		//get the directed forces from A to B and from B to A
		Vec3 forceAtoB = (-1 * m_fStiffness*lengthDifference)*lengthVectorAtoB;
		Vec3 forceBtoA = (-1 * m_fStiffness*lengthDifference)*lengthVectorBtoA;

		//write the directed forces into the masspoint struct
		masspoints[springs[i].masspoint1].springForce = forceAtoB;
		masspoints[springs[i].masspoint2].springForce = forceBtoA;
	}


	//Calculate final velocity for each mass point
	for (int i = 0; i < masspointCount; i++) {
		if (!masspoints[i].isFixed) {
			masspoints[i].velocity = masspoints[i].velocity + timeStep * masspoints[i].springForce / m_fMass;

			//std::cout << "Point " << i << " Pos: (" << masspoints[i].position.x << ", " << masspoints[i].position.y << ", " << masspoints[i].position.z << ") \n";
			//std::cout << "Point " << i << " Vel: (" << masspoints[i].velocity.x << ", " << masspoints[i].velocity.y << ", " << masspoints[i].velocity.z << ") \n";
		}
	}

}

void  MassSpringSystemSimulator::integrateLeapFrog(float timeStep) {

}



void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	for (int i = 0; i < masspointCount; i++) {
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 1 , 0.3*Vec3(1,1,1));
		DUC->drawSphere(masspoints[i].position, 0.05);
	}
	for (int i = 0; i < springCount; i++) {
		DUC->beginLine();
		DUC->drawLine(masspoints[springs[i].masspoint1].position, Vec3(1, 1, 1), masspoints[springs[i].masspoint2].position, Vec3(1, 1, 1));
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setUpDemo1() {
	//TODO: Create two points and simulate one step with any integration method, Switching doesnt work yet, simulation freezes
	//TODO: Print out initial position and end position
	reset();
	cout << "setup1";
	m_iIntegrator = 0;
	addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	addSpring(0, 1, 1);
	simulateTimestep(0.1f);
	std::cout << "EulerSolution:"<< "Point " << 1<< " Pos:" << masspoints[0].position.x << ", " << masspoints[0].position.y << ", " << masspoints[0].position.z << ") \n";
	std::cout << "Point " << 1 << " Vel: (" << masspoints[0].velocity.x << ", " << masspoints[0].velocity.y << ", " << masspoints[0].velocity.z << ") \n";
	std::cout << "Point " << 2 << " Pos:" << masspoints[1].position.x << ", " << masspoints[1].position.y << ", " << masspoints[1].position.z << ") \n";
	std::cout << "Point " << 2 << " Vel: (" << masspoints[1].velocity.x << ", " << masspoints[1].velocity.y << ", " << masspoints[1].velocity.z << ") \n";

	std::cout << "MidpointSolution:" << "Point " << 1 << " Pos:" << masspoints[0].position.x << ", " << masspoints[0].position.y << ", " << masspoints[0].position.z << ") \n";
	std::cout << "Point " << 1 << " Vel: (" << masspoints[0].velocity.x << ", " << masspoints[0].velocity.y << ", " << masspoints[0].velocity.z << ") \n";
	std::cout << "Point " << 2 << " Pos:" << masspoints[1].position.x << ", " << masspoints[1].position.y << ", " << masspoints[1].position.z << ") \n";
	std::cout << "Point " << 2 << " Vel: (" << masspoints[1].velocity.x << ", " << masspoints[1].velocity.y << ", " << masspoints[1].velocity.z << ") \n";
	

	cout << getNumberOfMassPoints();
	cout << getNumberOfSprings();
}

void MassSpringSystemSimulator::setUpDemo2() {
	//TODO: Set up regular simulation with two points, user can choose which integration method should be used, time step = 0.005
	//TODO: Pay attention that changing the integration method can or cannot reset the scene (idk what makes more sense)
	reset();
	cout << "setup2";
	//m_iIntegrator = 0;
	notifyMethodChanged(0);
	addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	addSpring(0, 1, 1);

	cout << getNumberOfMassPoints();
	cout << getNumberOfSprings();
}

void MassSpringSystemSimulator::setUpDemo3() {
	//TODO: Set up 10 Point, 10 SPring simulation, use can freely choose which integration method should be used
	//TODO: Let user choose time step freely
	//TODO: implement gravity, collision detection
	reset();
	cout << "setup3";
	//m_iIntegrator = 2;
	notifyMethodChanged(1);
	addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	addSpring(0, 1, 1);
	
	cout << getNumberOfMassPoints();
	cout << getNumberOfSprings();
}

void MassSpringSystemSimulator::setUpDemo4() {
	
	reset();
	cout << "setup4";
	//m_iIntegrator = 0;
	notifyMethodChanged(0);
	applyExternalForce(Vec3(0, -9.81f, 0)); //adding gravity
	

	addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 1), false);
	addMassPoint(Vec3(0, 2, 0), Vec3(1, 2, 0), false);
	addMassPoint(Vec3(1, 0, 0), Vec3(-1, 0, -1), false);
	addMassPoint(Vec3(0, 2, 1), Vec3(1, -2, 0), false);
	addMassPoint(Vec3(2, 0, 0), Vec3(-1, 0, 3), false);
	addMassPoint(Vec3(0, 2, 2), Vec3(1, 4, 0), false);
	addMassPoint(Vec3(3, 0, 0), Vec3(-1, 0, -3), false);
	addMassPoint(Vec3(0, 2, 3), Vec3(1, -4, 0), false);
	addMassPoint(Vec3(4, 0, 0), Vec3(-1, 0, 5), false);
	addMassPoint(Vec3(0, 2, 4), Vec3(1, 6, 0), false);


	addSpring(0, 1, 1);
	addSpring(2, 3, 1);
	addSpring(4, 5, 1);
	addSpring(6, 7, 1);
	addSpring(8, 9, 1);
	addSpring(10, 1, 1);
	addSpring(3, 5, 1);
	addSpring(2, 7, 1);
	addSpring(9, 4, 1);
	addSpring(8, 6, 1);

	cout << getNumberOfMassPoints();
	cout << getNumberOfSprings();

}