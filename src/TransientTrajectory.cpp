#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <ctime>




static std::vector<Vector3> TransitionPointGene(const Vector3 & PointA, const Vector3 & PointB, const Vector3 & PointC, const int & PointNo)
{
  // This function is used to generate transition points between two points (Edge points excluded).
  double alphaUnit = 1.0/(1.0 * PointNo + 1.0);
  std::vector<Vector3> TransitionPoints;
  TransitionPoints.reserve(2 * PointNo + 3);
  TransitionPoints.push_back(PointA);
  for (int i = 0; i < PointNo; i++)
  {
    double alpha_i = (1.0 * i + 1.0) * alphaUnit;
    Vector3 TransitionPoint = PointA + alpha_i * (PointB - PointA);
    TransitionPoints.push_back(TransitionPoint);
  }
  TransitionPoints.push_back(PointB);
  for (int i = 0; i < PointNo; i++)
  {
    double alpha_i = (1.0 * i + 1.0) * alphaUnit;
    Vector3 TransitionPoint = PointB + alpha_i * (PointC - PointB);
    TransitionPoints.push_back(TransitionPoint);
  }
  TransitionPoints.push_back(PointC);
  return TransitionPoints;
}

std::vector<Config> TransientTrajGene(const Robot & SimRobot, const int & LinkInfoIndex, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<double> & InitConfig, const Vector3 & InitPoint, const std::vector<double> & GoalConfig, const Vector3 & GoalPoint, ReachabilityMap & RMObject)
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

  // This part is used to visualization.
  std::vector<Vector3> CirclePointContact;
  CirclePointContact.reserve(2 + CircleSampleNo);
  CirclePointContact.push_back(InitPoint);
  CirclePointContact.push_back(GoalPoint);

  std::vector<Vector3> IntermediatePoint;
  IntermediatePoint.reserve(CircleSampleNo);
  std::vector<double> IntermediatePointDist;
  IntermediatePointDist.reserve(CircleSampleNo);

  for (int i = 0; i < CircleSampleNo; i++)
  {
    Vector3 RefPoint_new;
    RotMatrix.transformPoint(RefPoint, RefPoint_new);
    Vector3 CirclePoint = RefPoint_new + InitPoint;
    CirclePointContact.push_back(CirclePoint);
    RefPoint = RefPoint_new;
    double RefPointDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(CirclePoint);
    if(RefPointDist>0)
    {
      IntermediatePoint.push_back(CirclePoint);
      IntermediatePointDist.push_back(RefPointDist);
    }
  }
  Vector3Writer(CirclePointContact, "CirclePointContact");

  // Here we would to generate two straight lines connecting from robot end effector's initial point, intermediate point, and goal point.
  const int PointNo = 10;
  int HighIndex = std::distance(IntermediatePointDist.begin(), std::max_element(IntermediatePointDist.begin(), IntermediatePointDist.end()));
  std::vector<Vector3> TransitionPoints = TransitionPointGene(InitPoint, IntermediatePoint[HighIndex], GoalPoint, PointNo);

  std::vector<double> RefConfig(SimRobot.q.size()), OptConfig(SimRobot.q.size());
  for (int i = 0; i < SimRobot.q.size(); i++)
  {
    RefConfig[i] = SimRobot.q[i];
    OptConfig[i] = SimRobot.q[i];
  }

  string UserFilePath = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/user/hrp2/";

  for (int i = 0; i < TransitionPoints.size(); i++)
  {
    Vector3 RefPos = TransitionPoints[i];
    int Res = TransientOptFn(SimRobot, RefConfig, LinkInfoIndex, RefPos, RobotLinkInfo, RMObject, OptConfig, 0);
    string ConfigName = "TransientConfig" + std::to_string(i) +".config";
    RobotConfigWriter(OptConfig, UserFilePath, ConfigName);
    RefConfig = OptConfig;
  }

  int a = 1;



  // for (int i = 0; i < CirclePointContact.size(); i++)
  // {
  //   TransitionPoints.push_back(CirclePointContact[i]);
  // }
  //
  // Vector3Writer(TransitionPoints, "TransitionPoints");

}
