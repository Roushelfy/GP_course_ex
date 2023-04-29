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
		m_particleVel[i] += m_gravity * timestep;
		m_particlePos[i] += m_particleVel[i] * timestep;
	}
}
void FlipSimulator::pushParticlesApart(int numIters) {}
void FlipSimulator::handleParticleCollisions(Vec3 obstaclePos, float obstacleRadius, Vec3 obstacleVel) {
	double minx = -0.5 + m_h + m_particleRadius, maxx = 0.5 - m_h - m_particleRadius;
	double x, y, z;
	for (int i = 0; i < m_iNumSpheres; i++) {
		x = m_particlePos[i][0], y = m_particlePos[i][1], z = m_particlePos[i][2];
		if (x < minx) {
			m_particlePos[i][0] = minx;
			m_particleVel[i][0] = 0;
		}
		if (x > maxx) {
			m_particlePos[i][0] = maxx;
			m_particleVel[i][0] = 0;
		}
		if (y < minx) {
			m_particlePos[i][1] = minx;
			m_particleVel[i][1] = 0;
		}
		if (y > maxx) {
			m_particlePos[i][1] = maxx;
			m_particleVel[i][1] = 0;
		}
		if (z < minx) {
			m_particlePos[i][2] = minx;
			m_particleVel[i][2] = 0;
		}
		if (z > maxx) {
			m_particlePos[i][2] = maxx;
			m_particleVel[i][2] = 0;
		}
	}
}
void FlipSimulator::updateParticleDensity() {}

void FlipSimulator::transferVelocities(bool toGrid, float flipRatio) {
	std::vector<float> WpUp, Wp;
	double dx, dy, dz;
	int n = m_iCellY * m_iCellZ;
	int m = m_iCellZ;
	if (toGrid) {
		for (int i = 0; i < m_iNumCells; i++) {
			m_type[i] = m_s[i] == 0 ? SOLID_CELL : EMPTY_CELL;
		}
		for (int i = 0; i < m_iNumSpheres; i++) {
			int x = (m_particlePos[i].x + 0.5) / m_h;
			int y = (m_particlePos[i].y + 0.5) / m_h;
			int z = (m_particlePos[i].z + 0.5) / m_h;
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
	m_vel.clear(); m_vel.resize(m_iNumCells, Vec3());
	for (int k = 0; k < 3; k++) {
		WpUp.clear(); WpUp.resize(m_iNumCells, 0);
		Wp.clear(); Wp.resize(m_iNumCells, 0);
		for (int i = 0; i < m_iNumSpheres; i++) {
			double x = max(min(m_particlePos[i].x + 0.5, 1.0), (double)m_h);
			double y = max(min(m_particlePos[i].y + 0.5, 1.0), (double)m_h);
			double z = max(min(m_particlePos[i].z + 0.5, 1.0), (double)m_h);
			if (k == 0) {
				dx = 0, dy = 0.5 * m_h, dz = 0.5 * m_h;
			}
			else if (k == 1) {
				dx = 0.5 * m_h, dy = 0, dz = 0.5 * m_h;
			}
			else {
				dx = 0.5 * m_h, dy = 0.5 * m_h, dz = 0;
			}
			int x0 = min((int)floor((x - dx) / m_h), m_iCellX - 1);
			double tx = x - dx - x0 * m_h;
			int x1 = min(x0 + 1, m_iCellX - 1);
			int y0 = min((int)floor((y - dy) / m_h), m_iCellY - 1);
			double ty = y - dy - y0 * m_h;
			int y1 = min(y0 + 1, m_iCellY - 1);
			int z0 = min((int)floor((z - dz) / m_h), m_iCellZ - 1);
			double tz = z - dz - z0 * m_h;
			int z1 = min(z0 + 1, m_iCellZ - 1);

			double sx = m_h - tx;
			double sy = m_h - ty;
			double sz = m_h - tz;

			double d1, d2, d3, d4, d5, d6, d7, d8;
			d1 = sx * sy * sz;
			d2 = tx * sy * sz;
			d3 = tx * ty * sz;
			d4 = sz * ty * sz;
			d5 = sx * sy * tz;
			d6 = tx * sy * tz;
			d7 = tx * ty * tz;
			d8 = sz * ty * tz;

			int nr1, nr2, nr3, nr4, nr5, nr6, nr7, nr8;
			nr1 = x0 * n + y0 * m + z0;
			nr2 = x1 * n + y0 * m + z0;
			nr3 = x1 * n + y1 * m + z0;
			nr4 = x0 * n + y1 * m + z0;
			nr5 = x0 * n + y0 * m + z1;
			nr6 = x1 * n + y0 * m + z1;
			nr7 = x1 * n + y1 * m + z1;
			nr8 = x0 * n + y1 * m + z1;
			float pv = m_particleVel[i][k];
			if (toGrid) {
				WpUp[nr1] += pv * d1; Wp[nr1] += d1;
				WpUp[nr2] += pv * d2; Wp[nr1] += d2;
				WpUp[nr3] += pv * d3; Wp[nr1] += d3;
				WpUp[nr4] += pv * d4; Wp[nr1] += d4;
				WpUp[nr5] += pv * d5; Wp[nr1] += d5;
				WpUp[nr6] += pv * d6; Wp[nr1] += d6;
				WpUp[nr7] += pv * d7; Wp[nr1] += d7;
				WpUp[nr8] += pv * d8; Wp[nr1] += d8;
			}
			else {
				int offset = k == 0 ? n : k == 1 ? m : 1;
				int valid1, valid2, valid3, valid4, valid5, valid6, valid7, valid8;
				valid1 = m_type[nr1] != EMPTY_CELL || m_type[nr1 - offset] != EMPTY_CELL;
				valid2 = m_type[nr2] != EMPTY_CELL || m_type[nr2 - offset] != EMPTY_CELL;
				valid3 = m_type[nr3] != EMPTY_CELL || m_type[nr3 - offset] != EMPTY_CELL;
				valid4 = m_type[nr4] != EMPTY_CELL || m_type[nr4 - offset] != EMPTY_CELL;
				valid5 = m_type[nr5] != EMPTY_CELL || m_type[nr5 - offset] != EMPTY_CELL;
				valid6 = m_type[nr6] != EMPTY_CELL || m_type[nr6 - offset] != EMPTY_CELL;
				valid7 = m_type[nr7] != EMPTY_CELL || m_type[nr7 - offset] != EMPTY_CELL;
				valid8 = m_type[nr8] != EMPTY_CELL || m_type[nr8 - offset] != EMPTY_CELL;
				double d = valid1 * d1 + valid2 * d2 + valid3 * d3 + valid4 * d4 + valid5 * d5 + valid6 * d6 + valid7 * d7 + valid8 * d8;
				if (d > 0) {
					m_particleVel[i][k] = (valid1 * d1 * m_vel[nr1][k] + valid2 * d2 * m_vel[nr2][k] + valid3 * d3 * m_vel[nr3][k] + valid4 * d4 * m_vel[nr4][k]
						+ valid5 * d5 * m_vel[nr5][k] + valid6 * d6 * m_vel[nr6][k] + valid7 * d7 * m_vel[nr7][k] + valid8 * d8 * m_vel[nr8][k]) / d;
				}
			}
		}
		if (toGrid) {
			for (int i = 0; i < m_iNumCells; i++) {
				if (Wp[i] > 0) {
					m_pre_vel[i][k] = m_vel[i][k] = WpUp[i] / Wp[i];
				}
			}
			/*for (int i = 0; i < m_iCellX; i++) {
				for (int j = 0; j < m_iCellY; j++) {
					for (int k = 0; k < m_iCellZ; k++);
				}
			}*/
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