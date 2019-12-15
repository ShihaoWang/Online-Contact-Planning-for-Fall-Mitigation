#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <ctime>

#include "CppSpline/CatmullRom.h"

static Vector3 ContactRelocater(const Vector3 & ContactPoint)
{
  // This function is used to relocate contact point according to signed distance and contact normal.
  double Margin = 0.025;    //2.5cm
  Vector3 ContactPointNormal = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(ContactPoint);
  Vector3 Contact = ContactPoint + Margin * ContactPointNormal;
  double CurDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(Contact);
  while(CurDist<=Margin)
  {
    // This inner function  conducts an iterative "push" of this contact accoding to signed distance normal.
    ContactPointNormal = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(ContactPoint);
    Contact = ContactPoint + Margin * ContactPointNormal;
    CurDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(Contact);
  }
  return Contact;
}

static std::vector<double> CubicCurveCoeff1D(const double & p_init, const double & p_goal, const int & PointNo)
{
  // This function is used to generate the single dimenion point with a cubic curve.
  // f(s) = a*s^3 + b*s^2 + c*s + d
  double d = p_init;
  double c = 0.0;
  double a = 2.0 * (p_init - p_goal);
  double b = -1.5 * a;

  double sUnit = 1.0/(1.0 * PointNo + 1.0);
  std::vector<double> OneDValue(PointNo);
  for (int i = 0; i < PointNo; i++)
  {
    double s = (1.0 * i + 1.0) * sUnit;
    double f_i = a * s * s * s + b * s * s + c * s + d;
    OneDValue[i] = f_i;
  }
  return OneDValue;
}

static std::vector<Vector3> CubicCurvePos(const Vector3 & PointA, const Vector3 & PointB, const Vector3 & PointC, const int & PointNo)
{
  std::vector<Vector3> CubicCurvePoints;
  CubicCurvePoints.reserve(2 * PointNo + 3);
  CubicCurvePoints.push_back(PointA);

  std::vector<double> PointA2B_x = CubicCurveCoeff1D(PointA.x, PointB.x, PointNo);
  std::vector<double> PointA2B_y = CubicCurveCoeff1D(PointA.y, PointB.y, PointNo);
  std::vector<double> PointA2B_z = CubicCurveCoeff1D(PointA.z, PointB.z, PointNo);

  for (int i = 0; i < PointNo; i++)
  {
    CubicCurvePoints.push_back(Vector3(PointA2B_x[i], PointA2B_y[i], PointA2B_z[i]));
  }
  CubicCurvePoints.push_back(PointB);

  std::vector<double> PointB2C_x = CubicCurveCoeff1D(PointB.x, PointC.x, PointNo);
  std::vector<double> PointB2C_y = CubicCurveCoeff1D(PointB.y, PointC.y, PointNo);
  std::vector<double> PointB2C_z = CubicCurveCoeff1D(PointB.z, PointC.z, PointNo);

  for (int i = 0; i < PointNo; i++)
  {
    CubicCurvePoints.push_back(Vector3(PointB2C_x[i], PointB2C_y[i], PointB2C_z[i]));
  }
  CubicCurvePoints.push_back(PointC);
  return CubicCurvePoints;
}

// static std::vector<Vector3> CubicCurvePos(const Vector3 & PointA, const Vector3 & PointB, const Vector3 & PointC, const int & PointNo)
// {
//   // This function is used to calculate the cubic curve
//
// }


static std::vector<Vector3> TransitionPointGene(const Vector3 & PointA, const Vector3 & PointB, const Vector3 & PointC, const int & PointNo)
{
  // This function is used to generate transition points within these three points.
  double alphaUnit = 1.0/(1.0 * PointNo + 1.0);
  std::vector<Vector3> TransitionPoints;
  TransitionPoints.reserve(2 * PointNo + 3);
  TransitionPoints.push_back(PointA);
  for (int i = 0; i < PointNo; i++)
  {
    double alpha_i = (1.0 * i + 1.0) * alphaUnit;
    Vector3 TransitionPoint = PointA + alpha_i * (PointB - PointA);
    if(NonlinearOptimizerInfo::SDFInfo.SignedDistance(TransitionPoint)>-0.005)    // Here 0.005 is defined from Reachability Map function.
    {
      TransitionPoints.push_back(TransitionPoint);
    }
    else
    {
      TransitionPoint = ContactRelocater(TransitionPoint);
      TransitionPoints.push_back(TransitionPoint);
    }
  }
  TransitionPoints.push_back(PointB);
  for (int i = 0; i < PointNo; i++)
  {
    double alpha_i = (1.0 * i + 1.0) * alphaUnit;
    Vector3 TransitionPoint = PointB + alpha_i * (PointC - PointB);
    if(NonlinearOptimizerInfo::SDFInfo.SignedDistance(TransitionPoint)>-0.005)    // Here 0.005 is defined from Reachability Map function.
    {
      TransitionPoints.push_back(TransitionPoint);
    }
    else
    {
      TransitionPoint = ContactRelocater(TransitionPoint);
      TransitionPoints.push_back(TransitionPoint);
    }
  }
  TransitionPoints.push_back(PointC);

  return TransitionPoints;
}

static void SplinePiece1DObjGene(const double & sInit, const double & sGoal, const double & PosInit, const double & VelInit, const double & PosGoal, const double & VelGoal, double & a, double & b, double & c, double & d)
{
  // This function is used to generate the piecewiese cubic spline.
  double M00 = 3.0 * sInit * sInit;
  double M01 = 2.0 * sInit;
  double M02 = 1.0;

  double M10 = 3.0 * sGoal * sGoal;
  double M11 = 2.0 * sGoal;
  double M12 = 1.0;

  double M20 = (sInit * sInit * sInit - sGoal * sGoal * sGoal);
  double M21 = sInit * sInit - sGoal * sGoal;
  double M22 = sInit - sGoal;

  Vector3 Col1(M00, M10, M20);
  Vector3 Col2(M01, M11, M21);
  Vector3 Col3(M02, M12, M22);

  Matrix3 CubicMat(Col1, Col2, Col3);
  Vector3 CubicVec(VelInit, VelGoal, PosInit-PosGoal);

  Matrix3 CubicMatInv;
  CubicMat.getInverse(CubicMatInv);

  Vector3 abc = CubicMatInv * CubicVec;

  a = abc.x;
  b = abc.y;
  c = abc.z;
  d = PosInit - a * sInit * sInit * sInit - b * sInit * sInit - c * sInit;
  return;
}

static SplineInfo SplinePiece3DObjGene(const double & sInit, const double & sGoal, const Vector3 & PosInit, const Vector3 & NormalInit, const Vector3 & PosGoal, const Vector3 & NormalGoal)
{
  double ax, bx, cx, dx;
  SplinePiece1DObjGene(sInit, sGoal, PosInit.x, NormalInit.x, PosGoal.x, NormalGoal.x, ax, bx, cx, dx);

  double ay, by, cy, dy;
  SplinePiece1DObjGene(sInit, sGoal, PosInit.y, NormalInit.y, PosGoal.y, NormalGoal.y, ay, by, cy, dy);

  double az, bz, cz, dz;
  SplinePiece1DObjGene(sInit, sGoal, PosInit.z, NormalInit.z, PosGoal.z, NormalGoal.z, az, bz, cz, dz);

  Vector3 a(ax, ay, az);
  Vector3 b(bx, by, bz);
  Vector3 c(cx, cy, cz);
  Vector3 d(dx, dy, dz);

  SplineInfo SplineObj(sInit, sGoal, a, b, c, d);
  return SplineObj;
}

static Curve* CurveObjGene(const std::vector<Vector3> & Points, const int & sNumber)
{
  Curve* curve = new CatmullRom();
  curve->set_steps(sNumber);
  for (int i = 0; i < Points.size(); i++)
  {
    curve->add_way_point(SplineVector(Points[i].x, Points[i].y, Points[i].z));
  }
  return curve;
}

static std::vector<SplineInfo> SplineObjGene(const Vector3 & PosInit, const Vector3 & NormalInit, const Vector3 & PosGoal, const Vector3 & NormalGoal, const int & PointNo)
{
  // This function is used to generate the cubic spline for given robot's end effector path.
  const double scale = 0.5;
  SplineInfo BaseSpline = SplinePiece3DObjGene(0.0, 1.0, PosInit, scale * NormalInit, PosGoal, -scale * NormalGoal);

  // Here we have the basic spline objective.
  // Select Four Points
  Vector3 P1 = BaseSpline.SplinePosVector(0.0);
  Vector3 P2 = BaseSpline.SplinePosVector(0.25);
  Vector3 P3 = BaseSpline.SplinePosVector(0.5);
  Vector3 P4 = BaseSpline.SplinePosVector(1.0);


  // std::cout << "nodes: " << curve->node_count() << std::endl;
  // std::cout << "total length: " << curve->total_length() << std::endl;

  // for (int i = 0; i < curve->node_count(); ++i)
  // {
  //   std::cout << "node #" << i << ": " << curve->node(i).toString() << " (length so far: " << curve->length_from_starting_point(i) << ")" << std::endl;
  // }

  std::vector<Vector3> Points;
  Points.push_back(P1);
  Points.push_back(P2);
  Points.push_back(P3);
  Points.push_back(P4);

  Curve* curve = CurveObjGene(Points, 100);

  std::vector<Vector3> SplinePointUpdate;
  std::vector<Vector3> SplinePoints;
  for (int i = 0; i < curve->node_count(); i++)
  {
    Vector3 SplinePoint(curve->node(i).x, curve->node(i).y, curve->node(i).z);
    SplinePoints.push_back(SplinePoint);
    double SplinePointDis = NonlinearOptimizerInfo::SDFInfo.SignedDistance(SplinePoint);
    if(SplinePointDis<0)
    {
      // sUpdate.push_back(s);
      SplinePointUpdate.push_back(SplinePoint);
    }
  }
  Vector3Writer(SplinePoints, "TransitionPoints");    // Here we have already got the transition points.
  delete curve;

  // switch (sUpdate)
  // {
  //   case /* value */:
  // }
}

TrajInfo TransientTrajGene(const Robot & SimRobot, const int & LinkInfoIndex, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<double> & InitConfig, const Vector3 & PosInit, const Vector3 & PosGoal, ReachabilityMap & RMObject, int & TransFeasFlag)
{
  // This function is used to generate robot' tranistion trajecotries given initial configuration and final configuration.
  // A Hermite spline is constructed with the information of position and velocity.

  Vector3 NormalInit = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(PosInit);
  Vector3 NormalGoal = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(PosGoal);

  const int PointNo = 20;
  std::vector<SplineInfo> SplineObj = SplineObjGene(PosInit,  NormalInit, PosGoal, NormalGoal, PointNo);

  // std::vector<Vector3> TransitionPoints =  CubicCurvePos(PosInit, NormalInit, PosGoal, NormalGoal, PointNo);
  //
  // Vector3Writer(TransitionPoints, "TransitionPoints");    // Here we have already got the transition points.
  //
  // std::vector<double> RefConfig(SimRobot.q.size()), OptConfig(SimRobot.q.size());
  // for (int i = 0; i < SimRobot.q.size(); i++)
  // {
  //   RefConfig[i] = SimRobot.q[i];
  //   OptConfig[i] = SimRobot.q[i];
  // }

  // string UserFilePath = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/user/hrp2/";
  std::vector<Config> TransientTraj;
  // TransientTraj.reserve(TransitionPoints.size());

  // int TransitionPointIndex = 0;
  // while(TransitionPointIndex<TransitionPoints.size())
  // {
  //   Vector3 RefPos = TransitionPoints[TransitionPointIndex];
  //   int Res = TransientOptFn(SimRobot, RefConfig, LinkInfoIndex, RefPos, RobotLinkInfo, RMObject, OptConfig, 0);
  //   switch(Res)
  //   {
  //     case -1:
  //     {
  //       TransFeasFlag = 0;
  //       return TrajInfo(TransientTraj);
  //     }
  //     break;
  //     default:
  //     {
  //       TransFeasFlag = 1;
  //     }
  //     break;
  //   }
  //   // string ConfigName = "TransientConfig" + std::to_string(TransitionPointIndex) +".config";
  //   // RobotConfigWriter(OptConfig, UserFilePath, ConfigName);
  //   RefConfig = OptConfig;
  //   TransientTraj.push_back(Config(OptConfig));
  //   TransitionPointIndex++;
  // }

  return TrajInfo(TransientTraj);
}
