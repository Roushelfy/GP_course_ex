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
	for (auto& box : boxes) {
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		matrix4x4<double> scale, rotation, translation;
		scale = box.scale;
		rotation = box.Orientation.getRotMat();
		translation.initTranslation(box.Position.x, box.Position.y, box.Position.z);
		matrix4x4<double> Obj2WorldMatrix = scale * rotation * translation;
		DUC->drawRigidBody(Obj2WorldMatrix);
	}
}
void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
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
	return boxes.size();
}
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	return boxes[i].Position;
}
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	return boxes[i].Velocity;
}
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	return boxes[i].Angular_velocity;
}
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	boxes[i].forces.push_back(Force(loc, force));
}
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
	boxes.push_back(Rigid_box(position, size, mass));
}
void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	boxes[i].Orientation = orientation;
}
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	boxes[i].Velocity = velocity;
}

void RigidBodySystemSimulator::notifyGravityChanged(float gravity) {
}