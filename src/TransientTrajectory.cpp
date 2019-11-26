#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <ctime>

std::vector<Config> TransientTrajGene(const Robot & SimRobot, const int & LinkInfoIndex, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<double> & InitConfig, const Vector3 & InitPoint, const std::vector<double> & GoalConfig, const Vector3 & GoalPoint)
{
  // This function is used to generate robot' tranistion trajecotries given initial configuration and final configuration.
  double Ratio = 0.5;        // This ratio stands for height ration with respect to point distance.
  Vector3 Init2GoalVec = GoalPoint - InitPoint;
  double Init2GoalDist = sqrt(Init2GoalVec.x * Init2GoalVec.x + Init2GoalVec.y * Init2GoalVec.y + Init2GoalVec.z * Init2GoalVec.z);
  Vector3 gDirection(0.0, 0.0, -1.0);
  Vector3 RotAxis;
  RotAxis.setNormalized(Init2GoalVec);
  Vector3 RefUnit = cross(Init2GoalVec, gDirection);
  RefUnit.setNormalized(RefUnit);

  const int CircleSampleNo = 12;
  double RotAngleUnit = 2.0 * 3.1415926535/(CircleSampleNo * 1.0);
  AngleAxisRotation RotMatrix(RotAngleUnit, RotAxis);
  RefUnit = 0.5 * Ratio * Init2GoalDist * RefUnit;
  Vector3 RefPoint(RefUnit.x + 0.5 * Init2GoalVec.x, RefUnit.y + 0.5 * Init2GoalVec.y, RefUnit.z + 0.5 * Init2GoalVec.z);

  std::vector<Vector3> CirclePointContact;
  CirclePointContact.push_back(InitPoint);
  CirclePointContact.push_back(GoalPoint);

  for (int i = 0; i < CircleSampleNo; i++)
  {
    Vector3 RefPoint_new;
    RotMatrix.transformPoint(RefPoint, RefPoint_new);
    CirclePointContact.push_back(RefPoint_new+InitPoint);
    RefPoint = RefPoint_new;
  }

  Vector3Writer(CirclePointContact, "CirclePointContact");

  int a = 1;

}
