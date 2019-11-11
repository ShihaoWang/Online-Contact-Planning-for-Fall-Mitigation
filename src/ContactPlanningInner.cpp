#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include <omp.h>

static Vector3 StepIntegrator(double & Theta, double & Thetadot, const PIPInfo & PIPObj, double & dt)
{
  // This function is used to integrate robot's PIP dynamics.
  double L = PIPObj.L;
  double g = PIPObj.g;
  // Integration with the assumption that robot's acceleration remains to be constant during dt.
  double Thetaddot = g/L * sin(Theta);
  Theta = Theta + Thetadot * dt + 0.5 * Thetaddot * dt * dt;
  Thetadot = Thetadot + Thetaddot * dt;
  Vector3 COMPos = L * cos(Theta) * PIPObj.y_prime_unit - L * sin(Theta) * PIPObj.z_prime_unit + PIPObj.Intersection;
  return COMPos;
}

// This file saves functions needed for contact planning.
double CollisionTimeEstimator(const Vector3 & EdgeA, const Vector3 & EdgeB, const Vector3 & COMPos, const Vector3 & COMVel, SignedDistanceFieldInfo & SDFInfo)
{
  // This function is used to estimate the collision time for the robot.
  PIPInfo PIPObj = PIPGeneratorInner(EdgeA, EdgeB, COMPos, COMVel);
  // Then the robot is assumed to be rotating along PIP motion.
  // Whose equation of motion should be thetaddot = g*/L * sin(theta)

  // According to the observation, the robot's at most takes 2s to fall to the ground.
  double MaximumTime = 2.0;
  double CollisionTime = 0.0;
  double dt = 0.025;      // Each time step takes 0.025s.
  double Theta = PIPObj.theta;
  double Thetadot = PIPObj.thetadot;
  while (CollisionTime<MaximumTime)
  {
    Vector3 COMPosNew = StepIntegrator(Theta, Thetadot, PIPObj, dt);
    CollisionTime+=dt;
    // Computed COM position
    double CurrentDist = SDFInfo.SignedDistance(COMPosNew);
    if(CurrentDist<0)
    {
      break;
    }
  }
  // No matter what has happended, this while loop has to be terminated.
  int a = 1;
}
