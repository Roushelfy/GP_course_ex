#include "RigidBodySystemSimulator.h"
RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_iTestCase = 0;
}
const  char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Euler,Midpoint,complexEuler,complexMidpoint";
}
void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {

}
void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}
void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {

}
void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
}
void RigidBodySystemSimulator::notifyGravityChanged(float gravity) {
	if (m_iTestCase > 1)m_externalForce.y = -gravity;
}
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {

}
void RigidBodySystemSimulator::simulateTimestep(float timeStep) {

}
void RigidBodySystemSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
void RigidBodySystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// Specific Functions
int RigidBodySystemSimulator::getNumberOfRigidBodies() {

}
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {

}
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {

}
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {

}
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {

}
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {

}
void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {

}
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {

}