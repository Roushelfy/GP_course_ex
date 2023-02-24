#include "MassSpringSystemSimulator.h"
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_iTestCase = 0;
	m_fMass = 10;
	m_fInitialLength = 1;
	m_fStiffness = 40;
	m_iNumMass = 0;
	m_iNumSpring = 0;
}
const  char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Euler,Midpoint";
}
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
}
void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}
void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {

}
void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
}
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {

}
void MassSpringSystemSimulator::simulateTimestep(float timeStep) {

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
	return 0;
}
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {

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