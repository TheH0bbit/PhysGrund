#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	m_fMass = 10;
	m_fStiffness = 40;
	m_fDamping = 0;
	m_iIntegrator = 0;
}

const char * MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo1, Demo2, Demo3, Demo4";
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:break;
	case 2:break;
	default:break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	debug = false;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo1 !\n";
		initSystem(2, 1);

		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

		addSpring(0, 1, 1);
		
	case 1:
		cout << "Demo2 !\n";
		initSystem(2, 1);

		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

		addSpring(0, 1, 1);
		break;
	case 2:
		cout << "Demo3 !\n";
		initSystem(2, 1);

		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

		addSpring(0, 1, 1);
		break;
	case 3:
		cout << "Demo4 !\n";
		float x;
		float y;
		float z;
		float vx;
		float vy;
		float vz;
		int a;
		int b;
		int c; 

		initSystem(30, 30);

		addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), true);

		for (int i = 0; i < 29; i++)
		{
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			vx = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f) * 2.0f;
			vy = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f) * 2.0f;
			vz = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f) * 2.0f;
			addMassPoint(Vec3(x, y, z), Vec3(vx, vy, vz), false);
		}

		addSpring(0, 1, 1);
		for (int i = 0; i < 29; i++)
		{
			a = rand() % 30;
			b = rand() % 30;
			c = rand() % 4;
			
			while (a == b)
			{
				b = rand() % 30;
			}

			addSpring(a, b, c+1);
		}
		break;
	default:
		
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
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
		//inputWorld = inputWorld * inputScale;
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		//m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case 0: 
			integrateEuler(timeStep);

			//reset for midpoint
			initSystem(2, 1);
			addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
			addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
			addSpring(0, 1, 1);

			integrateMidpoint(timeStep);
		break;
	case 1: //eulerSim
		integrateEuler(timeStep);
		break;
	case 2: //midPointSim
		integrateMidpoint(timeStep);
	case 3: //complexSim
		integrateEuler(timeStep);
		break;
	default:
		break;
	}
}
	

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0: break;
	case 1:
		drawSpheresLines();
		break;
	case 2: 
		drawSpheresLines();
		break;
	case 3:
		drawSpheresLines();
		break;
	}
}

void MassSpringSystemSimulator::drawSpheresLines()
{
	DUC->setUpLighting(Vec3(1, 0, 0), Vec3(1, 0, 0), 0.5f, Vec3(1, 0, 0));
	for (int i = 0; i < massPoints.size(); i++)
	{
		DUC->drawSphere(massPoints[i].position, 0.05f);
	}
	DUC->beginLine();
	for (int i = 0; i < springs.size(); i++)
	{
		DUC->drawLine(massPoints[springs[i].p1].position, Vec3(0, 1, 0), massPoints[springs[i].p2].position, Vec3(0, 1, 0));
	}
	DUC->endLine();
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

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	int j = 0;
	for (int i = 0; i < massPoints.size(); i++)
	{
		if (!massPoints[i].isOpen)
			j++;
	}
	return j;
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	int j = 0;
	for (int i = 0; i < springs.size(); i++)
	{
		if (!springs[i].isOpen)
			j++;
	}
	return j;
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return massPoints[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return massPoints[index].velocity;
}

void MassSpringSystemSimulator::initSystem(int mp, int spring)
{
	massPoints.empty();
	massPoints.resize(mp);
	springs.empty();
	springs.resize(spring);
	
	for (int i = 0; i < mp; i++)
	{
		massPoints[i].isOpen = true;
	}
	for (int i = 0; i < spring; i++)
	{
		springs[i].isOpen = true;
	}
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	for (int i = 0; i < massPoints.size(); i++)
	{
		if (massPoints[i].isOpen)
		{
			massPoints[i].position = position;
			massPoints[i].velocity = Velocity;
			massPoints[i].isFixed = isFixed;
			massPoints[i].isOpen = false;
			return 1;
		}
	}
	return -1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	for (int i = 0; i < springs.size(); i++)
	{
		if (springs[i].isOpen)
		{
			springs[i].p1 = masspoint1;
			springs[i].p2 = masspoint2;
			springs[i].isOpen = false;
			springs[i].length = initialLength;
			return;
		}
	}
}

void MassSpringSystemSimulator::integrateEuler(float timeStep)
{
	for (int i = 0; i < massPoints.size(); i++)
	{
		massPoints[i].newPosition = massPoints[i].position + massPoints[i].velocity * timeStep;
	}

	for (int i = 0; i < springs.size(); i++)
	{
		if (!massPoints[springs[i].p1].isFixed || !massPoints[springs[i].p2].isFixed);
		{
			Vec3 dist = massPoints[springs[i].p1].position - massPoints[springs[i].p2].position;

			float force = -m_fStiffness * (norm(dist)-springs[i].length);
			Vec3 acc = force * getNormalized(dist) / m_fMass;

			if (!massPoints[springs[i].p1].isFixed)
				massPoints[springs[i].p1].velocity += timeStep * acc;
			if (!massPoints[springs[i].p2].isFixed)
				massPoints[springs[i].p2].velocity -= timeStep * acc;
		}
	}

	for (int i = 0; i < massPoints.size(); i++)
	{
		if (!massPoints[i].isFixed)
			massPoints[i].position = massPoints[i].newPosition;
		if (debug == false)
		{
			std::cout << "Midpoint: ";
			std::cout << "Point " << i << " Pos: (" << massPoints[i].position.x << ", " << massPoints[i].position.y << ", " << massPoints[i].position.z << ") \n";
			std::cout << "Point " << i << " Vel: (" << massPoints[i].velocity.x << ", " << massPoints[i].velocity.y << ", " << massPoints[i].velocity.z << ") \n";
		}
	}
	debug = true;
}

void MassSpringSystemSimulator::integrateMidpoint(float timeStep)
{
	//pos midstep
	for (int i = 0; i < massPoints.size(); i++)
	{
		massPoints[i].newPosition = massPoints[i].position + massPoints[i].velocity * timeStep / 2;
	}

	//vel midstep
	for (int i = 0; i < springs.size(); i++)
	{
		if (!massPoints[springs[i].p1].isFixed || !massPoints[springs[i].p2].isFixed);
		{
			Vec3 dist = massPoints[springs[i].p1].position - massPoints[springs[i].p2].position;

			float force = -m_fStiffness * (norm(dist) - springs[i].length);
			Vec3 acc = force * getNormalized(dist) / m_fMass;

			if (!massPoints[springs[i].p1].isFixed)
				massPoints[springs[i].p1].newVelocity = massPoints[springs[i].p1].velocity + timeStep/2 * acc;
			if (!massPoints[springs[i].p2].isFixed)
				massPoints[springs[i].p2].newVelocity = massPoints[springs[i].p2].velocity - timeStep/2 * acc;
		}
	}

	//end pos
	for (int i = 0; i < massPoints.size(); i++)
	{
		massPoints[i].position += massPoints[i].newVelocity * timeStep;
	}

	//end vel
	for (int i = 0; i < springs.size(); i++)
	{
		if (!massPoints[springs[i].p1].isFixed || !massPoints[springs[i].p2].isFixed);
		{
			Vec3 dist = massPoints[springs[i].p1].newPosition - massPoints[springs[i].p2].newPosition;

			float force = -m_fStiffness * (norm(dist) - springs[i].length);
			Vec3 acc = force * getNormalized(dist) / m_fMass;

			if (!massPoints[springs[i].p1].isFixed)
				massPoints[springs[i].p1].velocity = massPoints[springs[i].p1].velocity + timeStep * acc;
			if (!massPoints[springs[i].p2].isFixed)
				massPoints[springs[i].p2].velocity = massPoints[springs[i].p2].velocity - timeStep * acc;
		}
	}

	for (int i = 0; i < massPoints.size(); i++)
	{
		if (debug == false)
		{
			std::cout << "Midpoint: ";
			std::cout << "Point " << i << " Pos: (" << massPoints[i].position.x << ", " << massPoints[i].position.y << ", " << massPoints[i].position.z << ") \n";
			std::cout << "Point " << i << " Vel: (" << massPoints[i].velocity.x << ", " << massPoints[i].velocity.y << ", " << massPoints[i].velocity.z << ") \n";
		}
	}
	debug = true;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{

}
