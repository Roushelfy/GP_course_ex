#include "flip.h"
#include<cmath>
FlipSimulator::FlipSimulator() {
	m_iTestCase = 0;
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	m_gravity = { 0.0,-9.8,0 };
}
void FlipSimulator::integrateParticles(float timestep) {
	for (int i = 0; i < m_iNumSpheres; i++) {
		m_particlePos[i] += m_particleVel[i] * timestep;
		m_particleVel[i] += m_gravity * timestep;
	}
}
void FlipSimulator::pushParticlesApart(int numIters) {}
void FlipSimulator::handleParticleCollisions(Vec3 obstaclePos, float obstacleRadius, Vec3 obstacleVel) {}
void FlipSimulator::updateParticleDensity() {}

void FlipSimulator::transferVelocities(bool toGrid, float flipRatio) {
	std::vector<Vec3> WpUp, Wp, WiUi, Wi;
	WpUp.resize(m_iNumCells, Vec3());
	Wp.resize(m_iNumCells, Vec3());
	WiUi.resize(m_iNumCells, Vec3());
	Wi.resize(m_iNumCells, Vec3());
	float dx, dy, dz;
	int n = m_iCellY * m_iCellZ;
	int m = m_iCellZ;
	if (toGrid) {
		for (int i = 0; i < m_iNumCells; i++) {
			m_type[i] = m_s[i] == 0 ? SOLID_CELL : EMPTY_CELL;
		}
		for (int i = 0; i < m_iNumSpheres; i++) {
			int x = m_particlePos[i].x / m_h;
			int y = m_particlePos[i].y / m_h;
			int z = m_particlePos[i].z / m_h;
			if (x < 0)x = 0;
			if (y < 0)y = 0;
			if (z < 0)z = 0;
			if (x >= m_iCellX)x = m_iCellX - 1;
			if (y >= m_iCellY)y = m_iCellY - 1;
			if (z >= m_iCellZ)z = m_iCellZ - 1;
			int index = x * n + y * m + z;
			if (m_type[index] == EMPTY_CELL)
				m_type[index] = FLUID_CELL;
		}
	}
	for (int i = 0; i < 3; i++) {
		double x = max(min(m_particlePos[i].x, 1.0), 0.0);
		double y = max(min(m_particlePos[i].y, 1.0), 0.0);
		double z = max(min(m_particlePos[i].z, 1.0), 0.0);
		if (i == 0) {
			dx = 0, dy = 0.5 * m_h, dz = 0.5 * m_h;
			int x0 = min((int)floor((m_particlePos[i].x - dx) / m_h), m_iCellX - 1);
			double tx = x - x0 * m_h;
			int x1 = min(x0 + 1, m_iCellX - 1);
			int y0 = min((int)floor((m_particlePos[i].y - dy) / m_h), m_iCellY - 1);
			double ty = y - y0 * m_h;
			int y1 = min(y0 + 1, m_iCellY - 1);
			int z0 = min((int)floor((m_particlePos[i].z - dz) / m_h), m_iCellZ - 1);
			double tz = z - z0 * m_h;
			int z1 = min(z0 + 1, m_iCellZ - 1);

			double sx = m_h - tx;
			double sy = m_h - ty;
			double sz = m_h - tz;

			double d1, d2, d3, d4, d5, d6, d7, d8;
			d1 = sx * sy * sz;
			d2 = tx * sy * sz;
			d3 = tx * ty * sz;
			d4 =
		}
		else if (i == 1) {
			dx = 0.5 * m_h, dy = 0, dz = 0.5 * m_h;
		}
		else {
			dx = 0.5 * m_h, dy = 0.5 * m_h, dz = 0;
		}
	}
}
void FlipSimulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift) {}
void FlipSimulator::updateParticleColors() {}

// UI functions
const char* FlipSimulator::getTestCasesStr() {
	return "Flip";
}
void FlipSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
		setupScene(20);
	}
}
void FlipSimulator::reset() {}
void FlipSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	for (int i = 0; i < m_iNumSpheres; i++) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, m_particleColor[i]);
		DUC->drawSphere(m_particlePos[i], m_particleRadius);
	}
}
void FlipSimulator::notifyCaseChanged(int testCase) {}
void FlipSimulator::externalForcesCalculations(float timeElapsed) {}
void FlipSimulator::onClick(int x, int y) {}
void FlipSimulator::onMouse(int x, int y) {}
void FlipSimulator::notifyGravityChanged(float gravity) {
	m_gravity.y = -gravity;
}