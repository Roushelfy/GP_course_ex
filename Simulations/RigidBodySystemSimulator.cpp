#include "RigidBodySystemSimulator.h"
RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_iTestCase = 0;
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	coefficient = 1;
}
const  char* RigidBodySystemSimulator::getTestCasesStr() {
	return "SingleBody,TwoBody,ComplexScene";
}
void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	boxes.clear();
	switch (m_iTestCase)
	{
	case 0:
		addRigidBody(Vec3(0.5f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f));
		setAngularVelocityOf(0, Vec3(0.1, 0.1, 0.1));
		break;
	case 1:
		addRigidBody(Vec3(-1.0f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		setVelocityOf(0, Vec3(5, 0.0, 0));
		setOrientationOf(0, Quat(Vec3(0.0f, 1.0f, 1.0f), (float)(M_PI) * 0.5f));
		setAngularVelocityOf(0, Vec3(0.2, 0.4, 0.1));
		addRigidBody(Vec3(1.0f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		setVelocityOf(1, Vec3(-5, 0.0, 0));
		setOrientationOf(1, Quat(Vec3(1.0f, 0.0f, 0.0f), (float)(M_PI) * 0.5f));
		setAngularVelocityOf(1, Vec3(0.3, 0.1, 0.2));
		addboundingbox(2, 2, 2);
		break;
	case 2:
		addRigidBody(Vec3(-1.5f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		setVelocityOf(0, Vec3(5, 2, 0));
		setOrientationOf(0, Quat(Vec3(0.0f, 1.0f, 1.0f), (float)(M_PI) * 0.5f));
		setAngularVelocityOf(0, Vec3(0.2, 0.9, 0.1));
		addRigidBody(Vec3(1.2f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		setVelocityOf(1, Vec3(-5, -3, 0));
		setOrientationOf(1, Quat(Vec3(1.0f, 0.0f, 0.0f), (float)(M_PI) * 0.5f));
		setAngularVelocityOf(1, Vec3(3, 0.2, 0.2));
		addRigidBody(Vec3(0.0f, 1.4f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		setVelocityOf(2, Vec3(0.0, 10, -4));
		setOrientationOf(2, Quat(Vec3(0.0f, 1.0f, 1.0f), (float)(M_PI) * 0.5f));
		setAngularVelocityOf(2, Vec3(-0.7, -0.1, 0.03));
		addRigidBody(Vec3(0.0f, -1.1f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		setVelocityOf(3, Vec3(0.0, -10, 2));
		setOrientationOf(3, Quat(Vec3(0.1f, 0.2f, -0.1f), (float)(M_PI) * 0.5f));
		setAngularVelocityOf(3, Vec3(-0.1, 0.8, 0.2));
		addboundingbox(2, 2, 2);
		break;
	}
	for (auto& box : boxes) {
		matrix4x4<double> scale, rotation, translation;
		scale = box.scale;
		rotation = box.Orientation.getRotMat();
		translation.initTranslation(box.Position.x, box.Position.y, box.Position.z);
		matrix4x4<double> Obj2WorldMatrix = scale * rotation * translation;
		box.Obj2WorldMatrix = Obj2WorldMatrix;
	}
}
void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}
void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	for (auto& box : boxes) {
		if (box.Mass == 9998)
			continue;
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		DUC->drawRigidBody(box.Obj2WorldMatrix);
	}
}
void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
}


void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
	int n = getNumberOfRigidBodies();
	for (int i = 0; i < n; i++)
		for (int j = i + 1; j < n; j++) {
			if (boxes[i].Mass > 9000 && boxes[j].Mass > 9000) continue;
			CollisionInfo info = checkCollisionSAT(boxes[i].Obj2WorldMatrix, boxes[j].Obj2WorldMatrix);
			if (info.isValid) {
				Vec3 Xa = info.collisionPointWorld - boxes[i].Position;
				Vec3 Xb = info.collisionPointWorld - boxes[j].Position;
				double Vrel;
				Vec3 Va = cross(boxes[i].Angular_velocity, Xa) + boxes[i].Velocity;
				Vec3 Vb = cross(boxes[j].Angular_velocity, Xb) + boxes[j].Velocity;
				Vrel = dot(info.normalWorld, Va - Vb);
				if (Vrel <= 0) {
					matrix4x4<double> Ra = boxes[i].Orientation.getRotMat();
					matrix4x4<double> Rb = boxes[j].Orientation.getRotMat();
					matrix4x4<double> RaT = Ra, RbT = Rb;
					RaT.transpose(), RbT.transpose();
					matrix4x4<double> Ia = RaT * boxes[i].Inertial * Ra;
					matrix4x4<double> Ib = RbT * boxes[i].Inertial * Rb;
					Vec3 Impulse = -1 * (1 + coefficient) * Vrel * info.normalWorld;
					Impulse /= 1.0f / boxes[i].Mass + 1.0f / boxes[j].Mass + \
						dot(Ia.inverse().transformVector(cross(cross(Xa, info.normalWorld), Xa)) + Ib.inverse().transformVector(cross(cross(Xb, info.normalWorld), Xb)), info.normalWorld);
					//dot(Ia.inverse().transformVector(cross( Xa,cross( info.normalWorld,Xa))) + Ib.inverse().transformVector(cross(cross(Xb, info.normalWorld), Xb)), info.normalWorld);
					boxes[i].Velocity += Impulse / boxes[i].Mass;
					boxes[j].Velocity -= Impulse / boxes[j].Mass;
					boxes[i].Angular_velocity += Ia.inverse().transformVector(cross(Xa, Impulse));
					boxes[j].Angular_velocity -= Ib.inverse().transformVector(cross(Xb, Impulse));
				}
			}
		}
}
void RigidBodySystemSimulator::simulateTimestep(float timeStep) {
	for (auto& box : boxes) {
		if (box.Mass > 9000) continue;
		Vec3 netforce = Vec3();
		for (auto& force : box.forces)
			netforce += force.F;
		box.Position += timeStep * box.Velocity;
		box.Velocity += timeStep * netforce / box.Mass;
		matrix4x4<double> R = box.Orientation.getRotMat();
		Vec3 Nettorqe = Vec3();
		for (auto& force : box.forces)
			Nettorqe += cross((force.Position - box.Position), force.F);
		matrix4x4<double> RT = R;
		RT.transpose();
		matrix4x4<double> curI = RT * box.Inertial * R;
		box.Orientation += Quaternion<Real>(box.Angular_velocity.x * timeStep / 2, box.Angular_velocity.y * timeStep / 2, box.Angular_velocity.z * timeStep / 2, 0.0f) * box.Orientation;
		box.Orientation = box.Orientation.unit();
		box.Angular_velocity += timeStep * curI.inverse().transformVector(Nettorqe);
		matrix4x4<double> scale, rotation, translation;
		scale = box.scale;
		rotation = box.Orientation.getRotMat();
		translation.initTranslation(box.Position.x, box.Position.y, box.Position.z);
		matrix4x4<double> Obj2WorldMatrix = scale * rotation * translation;
		box.Obj2WorldMatrix = Obj2WorldMatrix;
		box.forces.clear();
	}
}
void RigidBodySystemSimulator::onClick(int x, int y) {
	m_mouse.x = x;
	m_mouse.y = y;
	//std::cout << x << ' ' << y << endl;
	int n = getNumberOfRigidBodies();
	//cout << n << endl;
	for (int i = 0; i < n; i++) {
		if (boxes[i].Mass > 9000) continue;
		boxes[i].forces.clear();
		applyForceOnBody(i, boxes[i].Position, Vec3(0.1 * (x - 600), 0.1 * (-y + 300), 0));
	}
}
void RigidBodySystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = m_trackmouse.x;
	m_oldtrackmouse.y = m_trackmouse.y;
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
void RigidBodySystemSimulator::setAngularVelocityOf(int i, Vec3 AngularVelocity) {
	boxes[i].Angular_velocity = AngularVelocity;
}
void RigidBodySystemSimulator::notifyGravityChanged(float gravity) {
	if (m_iTestCase == 0) return;
	int n = getNumberOfRigidBodies();
	for (int i = 0; i < n; i++) {
		if (boxes[i].Mass > 9000) continue;
		boxes[i].forces.clear();
		applyForceOnBody(i, boxes[i].Position, Vec3(0, -gravity * boxes[i].Mass, 0));
	}
}

void RigidBodySystemSimulator::addboundingbox(double x, double y, double z) {
	int NUM = 3;
	for (int i = -1; i <= y + 1; i++) {
		for (int j = -1; j <= z + 1; j++) {
			addRigidBody(Vec3(-x - 0.26, i - 0.5 * y, j - z * 0.5), Vec3(0.5f, 1.0, 1.0), 9999);
			addRigidBody(Vec3(x + 0.26, i - 0.5 * y, j - z * 0.5), Vec3(0.5f, 1.0, 1.0), 9999);
		}
	}
	for (int i = -1; i <= x + 1; i++) {
		for (int j = -1; j <= z + 1; j++) {
			addRigidBody(Vec3(i - x * 0.5, -y - 0.26, j - z * 0.5), Vec3(1.0, 0.5, 1.0), 9999);
			addRigidBody(Vec3(i - x * 0.5, y + 0.26, j - z * 0.5), Vec3(1.0, 0.5, 1.0), 9999);
		}
	}
	for (int i = -1; i <= x + 1; i++) {
		for (int j = -1; j <= y + 1; j++) {
			addRigidBody(Vec3(i - x * 0.5, j - y * 0.5, -z - 0.26), Vec3(1.0f, 1.0, 0.5), 9998);
			addRigidBody(Vec3(i - x * 0.5, j - y * 0.5, z + 0.26), Vec3(1.0f, 1.0, 0.5), 9999);
		}
	}
}