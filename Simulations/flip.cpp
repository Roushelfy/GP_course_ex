#include "flip.h"
#include<cmath>
#include<omp.h>

FlipSimulator::FlipSimulator() {
	omp_set_num_threads(16);
	m_iTestCase = 0;
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	m_fRatio = 0.7;
	m_useAffine = 0;
	m_freso = 22;
	m_gravity = { 0.0,-9.8,0 };
	m_lightdirection = { -0.5, -0.5, -0.5 };
}
void FlipSimulator::integrateParticles(float timestep) {
#pragma omp parallel for
	for (int i = 0; i < m_iNumSpheres; i++) {
		m_particleVel[i] += m_gravity * timestep;
		m_particlePos[i] += m_particleVel[i] * timestep;
	}
	obstaclePos += obstacleVel * timestep;
}
void FlipSimulator::pushParticlesApart(int numIters) {
	float colorDiffusionCoeff = 0.01f;
	vector<int> numcellParticles(m_iNumCells);
	int n = m_iCellY * m_iCellZ;
	int m = m_iCellZ;
	for (int i = 0; i < m_iNumSpheres; i++) {
		int x = (m_particlePos[i].x + 0.5) * m_fInvSpacing;
		int y = (m_particlePos[i].y + 0.5) * m_fInvSpacing;
		int z = (m_particlePos[i].z + 0.5) * m_fInvSpacing;
		if (x < 0)x = 0;
		if (y < 0)y = 0;
		if (z < 0)z = 0;
		if (x >= m_iCellX)x = m_iCellX - 1;
		if (y >= m_iCellY)y = m_iCellY - 1;
		if (z >= m_iCellZ)z = m_iCellZ - 1;
		int index = x * n + y * m + z;
		numcellParticles[index]++;
	}
	int first = 0;
	vector<int> firstcellParticles(m_iNumCells + 1);
	vector<int> ParticleCellids(m_iNumSpheres);
	for (int i = 0; i < m_iNumCells; i++) {
		first += numcellParticles[i];
		firstcellParticles[i] = first;
	}
	firstcellParticles[m_iNumCells] = first;
	for (int i = 0; i < m_iNumSpheres; i++) {
		int x = (m_particlePos[i].x + 0.5) * m_fInvSpacing;
		int y = (m_particlePos[i].y + 0.5) * m_fInvSpacing;
		int z = (m_particlePos[i].z + 0.5) * m_fInvSpacing;
		if (x < 0)x = 0;
		if (y < 0)y = 0;
		if (z < 0)z = 0;
		if (x >= m_iCellX)x = m_iCellX - 1;
		if (y >= m_iCellY)y = m_iCellY - 1;
		if (z >= m_iCellZ)z = m_iCellZ - 1;
		int index = x * n + y * m + z;
		firstcellParticles[index]--;
		ParticleCellids[firstcellParticles[index]] = i;
	}
	float minDist = 2.0 * m_particleRadius;
	float minDist2 = minDist * minDist;

	for (int iter = 0; iter < numIters; iter++) {
#pragma omp parallel for
		for (int i = 0; i < m_iNumSpheres; i++) {
			float px = m_particlePos[i].x;
			float py = m_particlePos[i].y;
			float pz = m_particlePos[i].z;
			int x = (px + 0.5) * m_fInvSpacing;
			int y = (py + 0.5) * m_fInvSpacing;
			int z = (pz + 0.5) * m_fInvSpacing;
			int x0 = max(x - 1, 0);
			int y0 = max(y - 1, 0);
			int z0 = max(z - 1, 0);
			int x1 = min(x + 1, m_iCellX - 1);
			int y1 = min(y + 1, m_iCellY - 1);
			int z1 = min(z + 1, m_iCellZ - 1);
			for (int xi = x0; xi < x1; xi++) {
				for (int yi = y0; yi < y1; yi++) {
					for (int zi = z0; zi < z1; zi++) {
						int index = xi * n + yi * m + zi;
						int first = firstcellParticles[index];
						int last = firstcellParticles[index + 1];
						for (int j = first; j < last; j++) {
							int id = ParticleCellids[j];
							if (id == i) continue;
							float qx = m_particlePos[id][0];
							float qy = m_particlePos[id][1];
							float qz = m_particlePos[id][2];
							float dx = qx - px;
							float dy = qy - py;
							float dz = qz - pz;
							float d2 = dx * dx + dy * dy + dz * dz;
							if (d2 > minDist2 || d2 == 0.0) continue;
							float d = sqrt(d2);
							float s = 0.5 * (minDist - d) / d;
							dx *= s, dy *= s, dz *= s;
							m_particlePos[i][0] -= dx;
							m_particlePos[i][1] -= dy;
							m_particlePos[i][2] -= dz;
							m_particlePos[id][0] += dx;
							m_particlePos[id][1] += dy;
							m_particlePos[id][2] += dz;
							for (int k = 0; k < 3; k++) {
								float color = (m_particleColor[i][k] + m_particleColor[id][k]) * 0.5;
								m_particleColor[i][k] += (color - m_particleColor[i][k]) * colorDiffusionCoeff;
								m_particleColor[id][k] += (color - m_particleColor[id][k]) * colorDiffusionCoeff;
							}
						}
					}
				}
			}
		}
	}
}
void FlipSimulator::handleParticleCollisions(Vec3 obstaclePos, float obstacleRadius, Vec3 obstacleVel) {
	float minx = -0.5 + m_h + m_particleRadius, maxx = (m_iCellX - 1) * m_h - 0.5 - m_particleRadius;
	float minDist = obstacleRadius + m_particleRadius;
	float minDist2 = minDist * minDist;
	float x, y, z;
	for (int i = 0; i < m_iNumSpheres; i++) {
		x = m_particlePos[i][0], y = m_particlePos[i][1], z = m_particlePos[i][2];
		float dx, dy, dz;
		dx = x - obstaclePos[0], dy = y - obstaclePos[1], dz = z - obstaclePos[2];
		float d2 = dx * dx + dy * dy + dz * dz;
		if (d2 < minDist2) {
			m_particleVel[i] = obstacleVel;
		}
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
void FlipSimulator::updateParticleDensity() {
	int n = m_iCellY * m_iCellZ;
	int m = m_iCellZ;
	float h2 = m_h * 0.5;
	std::fill(m_particleDensity.begin(), m_particleDensity.end(), 0.0);
	for (int i = 0; i < m_iNumSpheres; i++) {
		float x = max((float)min(m_particlePos[i].x + 0.5, 1.0 - m_h), (float)m_h);
		float y = max((float)min(m_particlePos[i].y + 0.5, 1.0 - m_h), (float)m_h);
		float z = max((float)min(m_particlePos[i].z + 0.5, 1.0 - m_h), (float)m_h);

		int x0 = min((int)floor((x - h2) * m_fInvSpacing), m_iCellX - 1);
		float tx = (x - h2 - x0 * m_h) * m_fInvSpacing;
		int x1 = min(x0 + 1, m_iCellX - 1);
		int y0 = min((int)floor((y - h2) * m_fInvSpacing), m_iCellY - 1);
		float ty = (y - h2 - y0 * m_h) * m_fInvSpacing;
		int y1 = min(y0 + 1, m_iCellY - 1);
		int z0 = min((int)floor((z - h2) * m_fInvSpacing), m_iCellZ - 1);
		float tz = (z - h2 - z0 * m_h) * m_fInvSpacing;
		int z1 = min(z0 + 1, m_iCellZ - 1);

		float sx = 1.0 - tx;
		float sy = 1.0 - ty;
		float sz = 1.0 - tz;

		float d1, d2, d3, d4, d5, d6, d7, d8;
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
		//if (x0 < m_iCellX && y0 < m_iCellY && z0 < m_iCellZ) 
		m_particleDensity[nr1] += d1;
		m_particleDensity[nr2] += d2;
		m_particleDensity[nr3] += d3;
		m_particleDensity[nr4] += d4;
		m_particleDensity[nr5] += d5;
		m_particleDensity[nr6] += d6;
		m_particleDensity[nr7] += d7;
		m_particleDensity[nr8] += d8;
	}
	if (m_particleRestDensity == 0.0) {
		float sum = 0;
		float fluidcells = 0;
		for (int i = 0; i < m_iNumCells; i++) {
			if (m_type[i] == FLUID_CELL) {
				sum += m_particleDensity[i];
				fluidcells += 1;
			}
		}
		if (fluidcells > 0) m_particleRestDensity = sum / fluidcells;
	}
}

void FlipSimulator::transferVelocities(bool toGrid, float flipRatio, bool useAffine) {
	std::vector<float> WpUp(m_iNumCells), Wp(m_iNumCells);
	float dx, dy, dz;
	int n = m_iCellY * m_iCellZ;
	int m = m_iCellZ;
	if (toGrid) {
		//m_pre_vel = m_vel;
		std::fill(m_vel.begin(), m_vel.end(), 0);
#pragma omp parallel for
		for (int i = 0; i < m_iNumCells; i++) {
			m_type[i] = m_s[i] == 0 ? SOLID_CELL : EMPTY_CELL;
		}
#pragma omp parallel for
		for (int i = 0; i < m_iNumSpheres; i++) {
			int x = (m_particlePos[i].x + 0.5) * m_fInvSpacing;
			int y = (m_particlePos[i].y + 0.5) * m_fInvSpacing;
			int z = (m_particlePos[i].z + 0.5) * m_fInvSpacing;
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
	//std::fill(m_vel.begin(), m_vel.end(), 0);
	for (int k = 0; k < 3; k++) {
		std::fill(Wp.begin(), Wp.end(), 0);
		std::fill(WpUp.begin(), WpUp.end(), 0);
#pragma omp parallel for
		for (int i = 0; i < m_iNumSpheres; i++) {
			float x = max((float)min(m_particlePos[i].x + 0.5, 1.0 - m_h), (float)m_h);
			float y = max((float)min(m_particlePos[i].y + 0.5, 1.0 - m_h), (float)m_h);
			float z = max((float)min(m_particlePos[i].z + 0.5, 1.0 - m_h), (float)m_h);
			if (k == 0) {
				dx = 0, dy = 0.5 * m_h, dz = 0.5 * m_h;
			}
			else if (k == 1) {
				dx = 0.5 * m_h, dy = 0, dz = 0.5 * m_h;
			}
			else {
				dx = 0.5 * m_h, dy = 0.5 * m_h, dz = 0;
			}
			int x0 = min((int)floor((x - dx) * m_fInvSpacing), m_iCellX - 1);
			float tx = (x - dx - x0 * m_h) * m_fInvSpacing;
			int x1 = min(x0 + 1, m_iCellX - 1);
			int y0 = min((int)floor((y - dy) * m_fInvSpacing), m_iCellY - 1);
			float ty = (y - dy - y0 * m_h) * m_fInvSpacing;
			int y1 = min(y0 + 1, m_iCellY - 1);
			int z0 = min((int)floor((z - dz) * m_fInvSpacing), m_iCellZ - 1);
			float tz = (z - dz - z0 * m_h) * m_fInvSpacing;
			int z1 = min(z0 + 1, m_iCellZ - 1);

			float sx = 1.0 - tx;
			float sy = 1.0 - ty;
			float sz = 1.0 - tz;

			float d1, d2, d3, d4, d5, d6, d7, d8;
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
				WpUp[nr2] += pv * d2; Wp[nr2] += d2;
				WpUp[nr3] += pv * d3; Wp[nr3] += d3;
				WpUp[nr4] += pv * d4; Wp[nr4] += d4;
				WpUp[nr5] += pv * d5; Wp[nr5] += d5;
				WpUp[nr6] += pv * d6; Wp[nr6] += d6;
				WpUp[nr7] += pv * d7; Wp[nr7] += d7;
				WpUp[nr8] += pv * d8; Wp[nr8] += d8;
			}
			else {
				int offset = k == 0 ? n : k == 1 ? m : 1;
				float valid1, valid2, valid3, valid4, valid5, valid6, valid7, valid8;
				valid1 = m_type[nr1] != EMPTY_CELL || m_type[nr1 - offset] != EMPTY_CELL;
				valid2 = m_type[nr2] != EMPTY_CELL || m_type[nr2 - offset] != EMPTY_CELL;
				valid3 = m_type[nr3] != EMPTY_CELL || m_type[nr3 - offset] != EMPTY_CELL;
				valid4 = m_type[nr4] != EMPTY_CELL || m_type[nr4 - offset] != EMPTY_CELL;
				valid5 = m_type[nr5] != EMPTY_CELL || m_type[nr5 - offset] != EMPTY_CELL;
				valid6 = m_type[nr6] != EMPTY_CELL || m_type[nr6 - offset] != EMPTY_CELL;
				valid7 = m_type[nr7] != EMPTY_CELL || m_type[nr7 - offset] != EMPTY_CELL;
				valid8 = m_type[nr8] != EMPTY_CELL || m_type[nr8 - offset] != EMPTY_CELL;

				float d = valid1 * d1 + valid2 * d2 + valid3 * d3 + valid4 * d4 + valid5 * d5 + valid6 * d6 + valid7 * d7 + valid8 * d8;
				if (d > 0) {
					float picV = (valid1 * d1 * m_vel[nr1][k] + valid2 * d2 * m_vel[nr2][k] + valid3 * d3 * m_vel[nr3][k] + valid4 * d4 * m_vel[nr4][k]
						+ valid5 * d5 * m_vel[nr5][k] + valid6 * d6 * m_vel[nr6][k] + valid7 * d7 * m_vel[nr7][k] + valid8 * d8 * m_vel[nr8][k]) / d;
					float corr = (valid1 * d1 * (m_vel[nr1][k] - m_pre_vel[nr1][k]) + valid2 * d2 * (m_vel[nr2][k] - m_pre_vel[nr2][k]) +
						valid3 * d3 * (m_vel[nr3][k] - m_pre_vel[nr3][k]) + valid4 * d4 * (m_vel[nr4][k] - m_pre_vel[nr2][k])
						+ valid5 * d5 * (m_vel[nr5][k] - m_pre_vel[nr5][k]) + valid6 * d6 * (m_vel[nr6][k] - m_pre_vel[nr6][k])
						+ valid7 * d7 * (m_vel[nr7][k] - m_pre_vel[nr7][k]) + valid8 * d8 * (m_vel[nr8][k] - m_pre_vel[nr8][k])) / d;
					float flipV = corr + pv;
					m_particleVel[i][k] = (1.0 - flipRatio) * picV + flipRatio * flipV;
				}
			}
		}
		if (toGrid) {
			for (int i = 0; i < m_iNumCells; i++) {
				if (Wp[i] > 0) {
					m_pre_vel[i][k] = m_vel[i][k] = WpUp[i] / Wp[i];
				}
				else {
					m_pre_vel[i][k] = m_vel[i][k] = WpUp[i];
				}
			}
			/*for (int i = 0; i < m_iCellX; i++) {
				for (int j = 0; j < m_iCellY; j++) {
					for (int k = 0; k < m_iCellZ; k++) {
						bool solid = m_type[i * n + j * m + k] == SOLID_CELL;
						if (solid)
							m_vel[i * n + j * m + k] = m_pre_vel[i * n + j * m + k];
					}
				}
			}*/
		}
	}

}

void FlipSimulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift) {
	//fill(m_p.begin(), m_p.end(), 0);
	int n = m_iCellY * m_iCellZ;
	int m = m_iCellZ;
	//int cp = m_particleDensity * m_h / dt;
	for (int iter = 0; iter < numIters; iter++) {
#pragma omp parallel for
		for (int i = 1; i < m_iCellX - 1; i++) {
			for (int j = 1; j < m_iCellY - 1; j++) {
				for (int k = 1; k < m_iCellZ - 1; k++) {
					int center = i * n + j * m + k;
					if (m_type[center] != FLUID_CELL) continue;
					int x0 = (i - 1) * n + j * m + k;
					int x1 = (i + 1) * n + j * m + k;
					int y0 = i * n + (j - 1) * m + k;
					int y1 = i * n + (j + 1) * m + k;
					int z0 = i * n + j * m + k - 1;
					int z1 = i * n + j * m + k + 1;
					int sx0 = m_s[x0];
					int sx1 = m_s[x1];
					int sy0 = m_s[y0];
					int sy1 = m_s[y1];
					int sz0 = m_s[z0];
					int sz1 = m_s[z1];
					float s = sx0 + sx1 + sy0 + sy1 + sz0 + sz1;
					if (s == 0.0) continue;
					float div = m_vel[x1][0] - m_vel[center][0] + m_vel[y1][1] - m_vel[center][1] + m_vel[z1][2] - m_vel[center][2];
					if (m_particleRestDensity > 0 && compensateDrift) {
						float k = 1;
						float compression = m_particleDensity[center] - m_particleRestDensity;
						if (compression > 0)
							div -= k * compression;
					}
					float p = div / s;
					p *= overRelaxation;
					m_vel[x1][0] -= sx1 * p;
					m_vel[center][0] += sx0 * p;
					m_vel[y1][1] -= sy1 * p;
					m_vel[center][1] += sy0 * p;
					m_vel[z1][2] -= sz1 * p;
					m_vel[center][2] += sz0 * p;

				}
			}
		}
	}
}
void FlipSimulator::updateParticleColors() {
	int n = m_iCellY * m_iCellZ;
	int m = m_iCellZ;
#pragma omp parallel for
	for (int i = 0; i < m_iNumSpheres; i++) {
		m_particleColor[i][0] = max(0.0, m_particleColor[i][0] - 0.1);
		m_particleColor[i][1] = max(0.0, m_particleColor[i][1] - 0.1);
		m_particleColor[i][2] = min(1.0, m_particleColor[i][2] + 0.1);
		float px = m_particlePos[i].x;
		float py = m_particlePos[i].y;
		float pz = m_particlePos[i].z;
		int x = (px + 0.5) * m_fInvSpacing;
		int y = (py + 0.5) * m_fInvSpacing;
		int z = (pz + 0.5) * m_fInvSpacing;
		int index = n * x + m * y + z;
		if (m_particleRestDensity > 0) {
			float realdensity = m_particleDensity[i] / m_particleRestDensity;
			if (realdensity < 0.01) {
				m_particleColor[i][0] = 0.8;
				m_particleColor[i][1] = 0.8;
				m_particleColor[i][2] = 1.0;
			}
		}
	}
}

// UI functions
const char* FlipSimulator::getTestCasesStr() {
	return "Flip";
}
void FlipSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
		setupScene(m_freso);
	}
}
void FlipSimulator::reset() {}
void FlipSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	for (int i = 0; i < 2; i++) {
		DUC->g_pEffectPositionNormal->SetLightDirection(i, m_lightdirection.toDirectXVector());
		DUC->g_pEffectPositionNormalColor->SetLightDirection(i, m_lightdirection.toDirectXVector());
	}
	for (int i = 0; i < m_iNumSpheres; i++) {
		DUC->setUpLighting(Vec3(), Vec3(1.0), 1000, m_particleColor[i]);
		DUC->drawSphere(m_particlePos[i], m_particleRadius);
	}
	DUC->setUpLighting(Vec3(), Vec3(1.0), 1000, { 1,0,0 });
	DUC->drawSphere(obstaclePos, obstacleRadius);
}
void FlipSimulator::notifyCaseChanged(int testCase) {}
void FlipSimulator::externalForcesCalculations(float timeElapsed) {
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		//Mat4 projection = DUC->g_camera.GetProjMatrix();
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.0009f;
		inputWorld = inputWorld * inputScale;
		obstaclePos = obstacleFinalPos + inputWorld;
		obstacleVel = inputWorld / timeElapsed;
	}
	else {
		obstacleFinalPos = obstaclePos;
		obstacleVel = Vec3();
	}
}
void FlipSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
void FlipSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
void FlipSimulator::notifyGravityChanged(float gravity) {
	m_gravity.y = -gravity;
}