#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include <omp.h>
#include "NonlinearOptimizerInfo.h"

/*
    This function computes the robot's whole-body configuration based on the inverted pendulum rotation motion around axis.
*/

static void StepIntegrator(double & Theta, double & Thetadot, Vector3 & COMPos, Vector3 & COMVel, const PIPInfo & PIPObj, const double & dt)
{
  // This function is used to integrate robot's PIP dynamics.
  double L = PIPObj.L;
  double g = PIPObj.g;
  // Integration with the assumption that robot's acceleration remains to be constant during dt.
  double Thetaddot = g/L * sin(Theta);
  Theta = Theta + Thetadot * dt + 0.5 * Thetaddot * dt * dt;
  Thetadot = Thetadot + Thetaddot * dt;
  COMPos = L * cos(Theta) * PIPObj.y_prime_unit - L * sin(Theta) * PIPObj.z_prime_unit + PIPObj.Intersection;
  Vector3 COMVelDirMag = -cross(PIPObj.x_prime_unit, COMPos - PIPObj.Intersection);
  Vector3 COMVelDir;
  COMVelDirMag.getNormalized(COMVelDir);
  COMVel = L * Thetadot * COMVelDir;
}

// This file saves functions needed for contact planning.
double CollisionTimeEstimator(const Vector3 & EdgeA, const Vector3 & EdgeB, const Vector3 & COMPos, const Vector3 & COMVel, SignedDistanceFieldInfo & SDFInfo, std::vector<Vector3> & COMPosTraj, std::vector<Vector3> & COMVelTraj, int & CollisionIndex, const double & dt)
{
  // This function is used to estimate the collision time for the robot.
  PIPInfo PIPObj = PIPGeneratorInner(EdgeA, EdgeB, COMPos, COMVel);
  // Then the robot is assumed to be rotating along PIP motion.
  // Whose equation of motion should be thetaddot = g*/L * sin(theta)

  // According to the observation, the robot's at most takes 2.5s to fall to the ground.
  double MaximumTime = 2.5;
  double CollisionTime = 0.0;
  double Theta = PIPObj.theta;
  double Thetadot = PIPObj.thetadot;
  int MaximumDataPoint = floor(MaximumTime/dt);
  CollisionIndex = 0;
  // std::vector<double> ThetaTraj, ThetadotTraj;
  COMPosTraj.reserve(MaximumDataPoint);
  COMVelTraj.reserve(MaximumDataPoint);
  while (CollisionTime<MaximumTime)
  {
    Vector3 COMPosNew, COMVelNew;
    StepIntegrator(Theta, Thetadot, COMPosNew, COMVelNew, PIPObj, dt);
    // ThetaTraj.push_back(Theta);
    // ThetadotTraj.push_back(Thetadot);
    COMPosTraj.push_back(COMPosNew);
    COMVelTraj.push_back(COMVelNew);
    CollisionTime+=dt;
    // Computed COM position
    double CurrentDist = SDFInfo.SignedDistance(COMPosNew);
    if(CurrentDist<0)
    {
      break;
    }
    CollisionIndex++;

  }
  // No matter what has happended, this while loop has to be terminated.
  return CollisionTime;
}

double ContactModiPreEstimation(Robot & SimRobot, const PIPInfo & PIPObj, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & RobotContactInfo, SignedDistanceFieldInfo & SDFInfo, int & FixerInfoIndex, std::vector<Vector3> & COMPosTraj, std::vector<Vector3> & COMVelTraj, const double & dt)
{
  Vector3 EdgeA = PIPObj.EdgeA;
  Vector3 EdgeB = PIPObj.EdgeB;

  int EdgeAEndInfoIndex = Edge2EndEffector(SimRobot, EdgeA, RobotLinkInfo);
  int EdgeBEndInfoIndex = Edge2EndEffector(SimRobot, EdgeB, RobotLinkInfo);

  // Here the high level contact planning leads to two consequences: Contact Modification and Contact Addition

  // 1. Contact Modification Analysis
  FixerInfoIndex = -1;
  Vector3 RotEdgeOri, RotEdgeGoal;
  if(EdgeAEndInfoIndex == EdgeBEndInfoIndex)
  {
    // This means that EdgeA and EdgeB are on the same end effector so the other three end effectors can be used for contact planning.
    FixerInfoIndex = EdgeAEndInfoIndex;
    RotEdgeOri = EdgeA;
    RotEdgeGoal = EdgeB;
  }
  else
  {
    // This indicates the current end is owned by two end effectors so we should be more careful about the selection of which end effector to be modified.
    // Basically the main idea is to alter the link where the rest of the link has a lower failure metric.
    Vector3 RotKeepBOri, RotKeepBGoal;
    double KeepBFixedCost = FailureMetricwithContactChange(SimRobot, EdgeAEndInfoIndex, RobotLinkInfo, RobotContactInfo, RotKeepBOri, RotKeepBGoal);
    Vector3 RotKeepAOri, RotKeepAGoal;
    double KeepAFixedCost = FailureMetricwithContactChange(SimRobot, EdgeBEndInfoIndex, RobotLinkInfo, RobotContactInfo, RotKeepAOri, RotKeepAGoal);

    if(KeepBFixedCost>KeepAFixedCost)
    {
      // Here A should be kept fixed.
      FixerInfoIndex = EdgeAEndInfoIndex;
      RotEdgeOri = RotKeepAOri;
      RotEdgeGoal = RotKeepAGoal;
    }
    else
    {
      // Here B should be kept fixed.
      FixerInfoIndex = EdgeBEndInfoIndex;
      RotEdgeOri = RotKeepBOri;
      RotEdgeGoal = RotKeepBGoal;
    }
  }
  /* Robot's COMPos and COMVel */
  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(SimRobot, COMPos, COMVel);
  std::vector<Vector3> PredCOMPosTraj, PredCOMVelTraj;
  int CollisionIndex;
  double CollisionTime = CollisionTimeEstimator(RotEdgeOri, RotEdgeGoal, COMPos, COMVel, SDFInfo, PredCOMPosTraj, PredCOMVelTraj, CollisionIndex, dt);

  COMPosTraj.reserve(CollisionIndex);
  COMVelTraj.reserve(CollisionIndex);
  for (int i = 0; i < CollisionIndex; i++)
  {
    COMPosTraj.push_back(PredCOMPosTraj[i]);
    COMVelTraj.push_back(PredCOMVelTraj[i]);
  }
  return CollisionTime;
}