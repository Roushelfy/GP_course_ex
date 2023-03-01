#include "MassSpringSystemSimulator.h"
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_iTestCase = 0;
	m_fMass = 10;
	m_fInitialLength = 1;
	m_fStiffness = 40;
	m_fDamping = 0;
	m_iNumMass = 0;
	m_iNumSpring = 0;
	m_externalForce = Vec3();
}
const  char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Euler,Midpoint";
}
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	//cout << "called" << endl;
	this->DUC = DUC;
	PositionOfMassPoint.clear();
	VelocityOfMassPoint.clear();
	IsFixedMassPoint.clear();
	NetForceOfMassPoint.clear();
	Spring.clear();
	m_iNumMass = 0;
	m_iNumSpring = 0;
	switch (m_iTestCase)
	{
	case 0:
	case 1:
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), 0);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), 0);
		addSpring(0, 1, m_fInitialLength);
		break;
	case 2:break;
	default:break;
	}
}
void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}
void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {

	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(0, 0, 1));
	for (int i = 0; i < m_iNumMass; i++) {
		//cout << i << ' ' << "pos" << endl;
		//cout << PositionOfMassPoint[i] << endl;
		//cout << "vel" << endl;
		//cout << VelocityOfMassPoint[i] << endl;
		DUC->drawSphere(getPositionOfMassPoint(i), 0.05f);
	}
	for (int i = 0; i < m_iNumSpring; i++) {
		//cout << i << ' ' << "spring" << endl;
		//cout << Spring[i].first << ' ' << Spring[i].second << endl;
		DUC->beginLine();
		DUC->drawLine(getPositionOfMassPoint(Spring[i].first), Vec3(0, 1, 0), getPositionOfMassPoint(Spring[i].second), Vec3(0, 1, 0));
		DUC->endLine();
	}
}
void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
}
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	//cout << "called!" << endl;
	for (int i = 0; i < m_iNumMass; i++)
	{
		NetForceOfMassPoint[i] = m_externalForce;
	}
	for (int i = 0; i < m_iNumSpring; i++)
	{
		int i1 = Spring[i].first, i2 = Spring[i].second;
		Vec3 x1, x2, v1, v2;
		x1 = PositionOfMassPoint[i1], x2 = PositionOfMassPoint[i2];
		v1 = VelocityOfMassPoint[i1], v2 = VelocityOfMassPoint[i2];
		Vec3 pos = x2 - x1;
		double curlen = sqrt(pos.squaredDistanceTo(Vec3()));
		pos = pos / curlen;
		Vec3 SpringForce1, SpringForce2, Damping1, Damping2;
		SpringForce1 = m_fStiffness * (curlen - m_fInitialLength) * pos;
		SpringForce2 = -SpringForce1;
		Damping1 = m_fDamping * dot(pos, v2 - v1) * pos;
		Damping2 = -Damping1;
		NetForceOfMassPoint[i1] += SpringForce1 + Damping1;
		NetForceOfMassPoint[i2] += SpringForce2 + Damping2;
	}
}
void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	if (m_iTestCase == 0)
		for (int i = 0; i < m_iNumMass; i++)
		{
			if (IsFixedMassPoint[i]) continue;
			PositionOfMassPoint[i] += timeStep * VelocityOfMassPoint[i];
			VelocityOfMassPoint[i] += timeStep * NetForceOfMassPoint[i] / m_fMass;
		}
	else if (m_iTestCase == 1)
	{
		vector<Vec3> storex, storev;
		storex = PositionOfMassPoint;
		storev = VelocityOfMassPoint;
		for (int i = 0; i < m_iNumMass; i++)
		{
			if (IsFixedMassPoint[i]) continue;
			PositionOfMassPoint[i] += 0.5f * timeStep * VelocityOfMassPoint[i];
			VelocityOfMassPoint[i] += 0.5f * timeStep * NetForceOfMassPoint[i] / m_fMass;
		}
		externalForcesCalculations(0.5 * timeStep);
		for (int i = 0; i < m_iNumMass; i++)
		{
			if (IsFixedMassPoint[i]) continue;
			PositionOfMassPoint[i] = storex[i] + timeStep * VelocityOfMassPoint[i];
			VelocityOfMassPoint[i] = storev[i] + timeStep * NetForceOfMassPoint[i] / m_fMass;
		}
	}

}
void MassSpringSystemSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
void MassSpringSystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// Specific Functions
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
	m_iNumMass++;
	PositionOfMassPoint.push_back(position);
	VelocityOfMassPoint.push_back(Velocity);
	IsFixedMassPoint.push_back(isFixed);
	NetForceOfMassPoint.push_back(Vec3());
	return 0;
}
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	m_iNumSpring++;
	Spring.push_back({ masspoint1,masspoint2 });
	assert(initialLength == m_fInitialLength);
}
int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return m_iNumMass;
}
int MassSpringSystemSimulator::getNumberOfSprings() {
	return m_iNumSpring;
}
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return PositionOfMassPoint[index];
}
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return VelocityOfMassPoint[index];
}
void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	m_externalForce = force;
}