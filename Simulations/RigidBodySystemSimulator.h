#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 
#include "collisionDetect.h"

#define TESTCASEUSEDTORUNTEST 2
struct Force {
	Vec3 Position;
	Vec3 F;
	Force(Vec3 P, Vec3 f) :Position(P), F(f) {}
};
struct Rigid_box {
	Vec3 Size;
	matrix4x4<double> scale;
	int Mass;
	Vec3 Position;
	Vec3 Velocity;
	Quat Orientation;
	Vec3 Angular_velocity;
	matrix4x4<double> Inertial;
	matrix4x4<double> Obj2WorldMatrix;
	vector<Force> forces;
	Rigid_box(Vec3 pos, Vec3 size, int mass) :Position(pos), Size(size), Mass(mass) {
		double w2, h2, d2, arg;
		w2 = size.x * size.x, h2 = size.y * size.y, d2 = size.z * size.z;
		arg = 1.0f / 12 * Mass;
		Inertial = matrix4x4<double>(arg * (h2 + d2), 0, 0, 0, 0, arg * (d2 + w2), 0, 0, 0, 0, arg * (w2 + h2), 0, 0, 0, 0, 1);
		scale.initScaling(Size.x, Size.y, Size.z);
		Velocity = Vec3();
		Angular_velocity = Vec3();
		Orientation = Quat();
		Obj2WorldMatrix = matrix4x4<double>();
	}
};

class RigidBodySystemSimulator :public Simulator {
public:
	// Construtors
	RigidBodySystemSimulator();

	// Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void notifyGravityChanged(float gravity);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addboundingbox(double x, double y, double z);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
	void setAngularVelocityOf(int i, Vec3 AngularVelocity);
private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	vector<Rigid_box> boxes;
	Vec3 m_externalForce;
	double coefficient;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif