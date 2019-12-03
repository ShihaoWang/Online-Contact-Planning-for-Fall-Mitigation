// This function is used to calculate the task space controller for a certain contact to be established.
#include "NonlinearOptimizerInfo.h"
#include "CommonHeader.h"
#include <random>
#include "gurobi_c++.h"

static std::vector<Vector3> ConeAllUnits;
static std::vector<Matrix> ActJacobians;
static int EdgeNo;
static int DOF;
static std::vector<LinkInfo> RobotLinkInfo;
static std::vector<ContactStatusInfo> RobotContactInfo;
static std::vector<Vector3> ContactPositionsRef;    // This vector sves robot's reference contact positions.
static std::vector<Vector3> ContactPositions;       // This vector saves robot's current contact positions.
static std::vector<Vector3> ContactVelocities;      // This vector saves robot's current contact velocities.

/*
  A general task-space cost function is written to be
*/
static double Kpos = 0.0;

static double Kvel = 20.0;
static double Kqddot = 5.0;

static double Ku = 0.0;
static double Kf = 0.0;
static double Keta = 0.1;

static double Kp = 0.0;
static double Kd = 2.5;

static double ConAccTol = 1.0;

static double epsTol = 1e-8;

static double InfVal = 10000;
