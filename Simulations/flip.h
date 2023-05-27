#ifndef FLIP_H
#define FLIP_H
#include "Simulator.h"
const int EMPTY_CELL = 0; const int FLUID_CELL = 1; const int SOLID_CELL = 2;
typedef vector3Dim<Vec3> Mat3;
class FlipSimulator :public Simulator {
public:
	// Construtors
	FlipSimulator();

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	float m_fForceScaling;
	// FLIP/PIC ratio
	float m_fRatio;
	float m_freso;
	bool m_useAffine;
	// grid property
	//int obstaclemode;
	int m_iCellX;
	int m_iCellY;
	int m_iCellZ;
	float m_h; 			 // grid spacing, m_h = 1.0 / (m_iCellX-1)
	float m_fInvSpacing; // grid inverse spacing, m_fInvSpacing = 1.0/m_h
	int m_iNumCells;	 // m_iCellX * m_iCellY * m_iCellZ

	// particle property
	int m_iNumSpheres;
	float m_particleRadius;
	Vec3 m_gravity;
	Vec3 m_lightdirection;
	// particle data arrays
	std::vector<Vec3> m_particlePos;		// Particle Positions
	std::vector<Vec3> m_particleColor;		// Particle Color for visualization
	std::vector<Vec3> m_particleVel;		// Particle Velocity
	Vec3 obstaclePos;     // obstacle can be moved with mouse, as a user interaction
	Vec3 obstacleFinalPos;
	Vec3 obstacleVel;
	float obstacleRadius;
	// grid data arrays
	std::vector<Vec3>  m_vel;	  	// Velocity array
	std::vector<Mat3>  m_c;
	std::vector<Vec3>  m_pre_vel; 	// Hold the previous velocity for flip update
	std::vector<float> m_p; 		// Pressure array
	std::vector<float> m_s; 		// 0.0 for solid cells, 1.0 for fluid cells, used to update m_type
	std::vector<int>  m_type; 		// Flags array (const int EMPTY_CELL = 0; const int FLUID_CELL = 1; const int SOLID_CELL = 2;)
	// m_type = SOLID_CELL if m_s == 0.0; 
	// m_type = FLUID_CELL if has particle and m_s == 1; 
	// m_type = EMPTY_CELL if has No particle and m_s == 1; 
	std::vector<float> m_particleDensity;	// Particle Density per cell, saved in the grid cell
	float m_particleRestDensity;

	// Simulation Functions
	void integrateParticles(float timeStep);
	void pushParticlesApart(int numIters);
	void handleParticleCollisions(Vec3 obstaclePos, float obstacleRadius, Vec3 obstacleVel);
	void updateParticleDensity();

	void transferVelocities(bool toGrid, float flipRatio, bool useAffine);
	void solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift);
	void updateParticleColors();

	// UI functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	void notifyGravityChanged(float gravity);
	// two given functions:
	void simulateTimestep(float dt) {
		int numSubSteps = 1;
		int numParticleIters = 2;
		int numPressureIters = 30;
		bool separateParticles = true;
		float overRelaxation = 1.9;
		bool compensateDrift = true;

		float flipRatio = m_fRatio; // 0.95f;
		float sdt = dt / numSubSteps;

		for (int step = 0; step < numSubSteps; step++) {
			integrateParticles(sdt);
			if (separateParticles)
				pushParticlesApart(numParticleIters);
			handleParticleCollisions(obstaclePos, obstacleRadius, obstacleVel);
			transferVelocities(true, flipRatio, m_useAffine);
			updateParticleDensity();
			solveIncompressibility(numPressureIters, sdt, overRelaxation, compensateDrift);
			transferVelocities(false, flipRatio, m_useAffine);
		}

		updateParticleColors();
	}

	// ui functions
	void setupScene(int res)
	{// an example to set up a breaking dam scene
		float tankHeight = 1.0;
		float tankWidth = 1.0;
		float tankDepth = 1.0;
		//obstaclemode = 0;
		float _h = tankHeight / res;
		float point_r = 0.3 * _h;	// particle radius w.r.t. cell size

		float relWaterHeight = 0.8;
		float relWaterWidth = 0.6;
		float relWaterDepth = 0.6;
		obstaclePos = Vec3(0.0, 0.3, 0.0);     // obstacle can be moved with mouse, as a user interaction
		obstacleFinalPos = Vec3(0.0, 0.3, 0.0);
		obstacleVel = Vec3(0.0f);
		obstacleRadius = 0.1;
		// dam break
		// compute number of particles	
		float dx = 2.0 * point_r;
		float dy = sqrt(3.0) / 2.0 * dx;
		float dz = dx;

		int numX = floor((relWaterWidth * tankWidth - 2.0 * _h - 2.0 * point_r) / dx);
		int numY = floor((relWaterHeight * tankHeight - 2.0 * _h - 2.0 * point_r) / dy);
		int numZ = floor((relWaterDepth * tankDepth - 2.0 * _h - 2.0 * point_r) / dz);

		// update object member attributes
		m_iNumSpheres = numX * numY * numZ;
		m_iCellX = res + 1;
		m_iCellY = res + 1;
		m_iCellZ = res + 1;
		m_h = 1.0 / float(res);
		m_fInvSpacing = float(res);
		m_iNumCells = m_iCellX * m_iCellY * m_iCellZ;
		m_particleRadius = 0.3 * point_r;

		// update particle array
		m_particlePos.clear(); m_particlePos.resize(m_iNumSpheres, Vec3(0.0f));
		m_particleColor.clear(); m_particleColor.resize(m_iNumSpheres, Vec3(0.0f, 0.0f, 1.0f));
		m_particleVel.clear(); m_particleVel.resize(m_iNumSpheres, Vec3(0.0f));

		// update grid array
		m_vel.clear(); m_vel.resize(m_iNumCells, Vec3(0.0f));
		m_pre_vel.clear(); m_pre_vel.resize(m_iNumCells, Vec3(0.0f));
		m_p.clear();  m_p.resize(m_iNumCells, 0.0);
		m_s.clear(); m_s.resize(m_iNumCells, 0.0);
		m_type.clear(); m_type.resize(m_iNumCells, 0);
		m_particleDensity.clear(); m_particleDensity.resize(m_iNumCells, 0.0f);

		// the rest density can be assigned after scene initialization
		m_particleRestDensity = 0.0;

		// create particles
		int p = 0;
		for (int i = 0; i < numX; i++) {
			for (int j = 0; j < numY; j++) {
				for (int k = 0; k < numZ; k++) {
					m_particlePos[p++] = Vec3(m_h + point_r + dx * i + (j % 2 == 0 ? 0.0 : point_r), m_h + point_r + dy * j, m_h + point_r + dz * k + (j % 2 == 0 ? 0.0 : point_r)) + Vec3(-0.5f);
				}
			}
		}
		// setup grid cells for tank
		int n = m_iCellY * m_iCellZ;
		int m = m_iCellZ;

		for (int i = 0; i < m_iCellX; i++) {
			for (int j = 0; j < m_iCellY; j++) {
				for (int k = 0; k < m_iCellZ; k++) {
					float s = 1.0;	// fluid
					if (i == 0 || i == m_iCellX - 1 || j == 0 || k == 0 || k == m_iCellZ - 1)
						s = 0.0f;	// solid
					m_s[i * n + j * m + k] = s;
				}
			}
		}
		// set others, e.g.,
		//setObstacle(3.0, 2.0, true);

	}

};
#endif