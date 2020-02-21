#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <ctime>
#include "Splines.h"

using namespace SplineLib;

static Vector3 ShiftPointFunc(const Vector3 & ShiftPoint, bool & Flag)
{
  // This function is used to relocate contact point according to signed distance and contact normal.
  // Here the signed distance of shift point should be negative.
  double Margin = -NonlinearOptimizerInfo::SDFInfo.SignedDistance(ShiftPoint);
  Vector3 ShiftPointNew = ShiftPoint;
  Vector3 ShiftPointNormal;
  double CurDist = 0.0;

  Flag = false;
  const int TotalNo = 5;
  int CurrentNo = 0;
  while(CurrentNo<=TotalNo)
  {
    // This inner function conducts an iterative "push" of this contact accoding to signed distance normal.
    ShiftPointNormal = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(ShiftPointNew);
    ShiftPointNew = ShiftPointNew + Margin * ShiftPointNormal;
    CurDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(ShiftPointNew);
    if(CurDist>0.0)
    {
      Flag = true;
      break;
    }
    CurrentNo++;
  }
  return ShiftPointNew;
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

static std::vector<Vector3> BasePointsGene(const Vector3 & PosInit, const Vector3 & NormalInit, const Vector3 & PosGoal, const Vector3 & NormalGoal)
{
  // This function is used to generate the cubic spline for given robot's end effector path.
  const double scale = 0.5;
  SplineInfo BaseSpline = SplinePiece3DObjGene(0.0, 1.0, PosInit, scale * NormalInit, PosGoal, -scale * NormalGoal);
  const int segmentNo = 3;
  double sUnit = 1.0/(1.0 * segmentNo);
  std::vector<Vector3> BasePoints(segmentNo+1);
  for (int i = 0; i < segmentNo + 1; i++)
  {
    double s = 1.0 * i * sUnit;
    Vector3 BasePoint = BaseSpline.SplinePosVector(s);
    BasePoints[i] = BasePoint;
  }
  return BasePoints;
}

static double SelfCollisionDist(const Robot & SimRobot, SelfLinkGeoInfo & SelfLinkGeoObj, ReachabilityMap & RMObject, const int & LinkInfoIndex, const Vector3 & PosInit, const Vector3 & PosGoal)
{
  // This function helps calculate robot's clearance for collision-avoidance
  std::vector<int> EndEffectorLinkIndex = RMObject.EndEffectorLink2Pivotal[LinkInfoIndex];
  double DistTol = 100000000.0;
  // for (int i = 6; i < SimRobot.q.size(); i++)
  // {
  //   if(std::find(EndEffectorLinkIndex.begin(), EndEffectorLinkIndex.end(), i) == EndEffectorLinkIndex.end())
  //   {
  //     Frame3D LinkLocalFrame = SimRobot.links[i].T_World;
  //     Vector3 PosInitLocal, PosGoalLocal;
  //     LinkLocalFrame.mulPointInverse(PosInit, PosInitLocal);
  //     LinkLocalFrame.mulPointInverse(PosGoal, PosGoalLocal);
  //     double PosInitLocalDist = SelfLinkGeoObj.LinkSDFs[i-6].AsImplicitSurface().TrilinearInterpolate(PosInitLocal);
  //     double PosGoalLocalDist = SelfLinkGeoObj.LinkSDFs[i-6].AsImplicitSurface().TrilinearInterpolate(PosGoalLocal);
  //     double PosCenterLocalDist = SelfLinkGeoObj.LinkSDFs[i-6].AsImplicitSurface().TrilinearInterpolate(Vector3(0.0, 0.0, 0.0));
  //
  //     // PosInitLocalDist = SelfLinkGeoObj.LinkSDFs[i].AsImplicitSurface().TrilinearInterpolate(PosInitLocal);
  //     // PosGoalLocalDist = SelfLinkGeoObj.LinkSDFs[i].AsImplicitSurface().TrilinearInterpolate(PosGoalLocal);
  //     if(PosInitLocalDist<DistTol) DistTol = PosInitLocalDist;
  //     if(PosGoalLocalDist<DistTol) DistTol = PosGoalLocalDist;
  //   }
  // }
  return DistTol;
}

static void SelfCollisionFn(const Robot & SimRobot, SelfLinkGeoInfo & SelfLinkGeoObj, ReachabilityMap & RMObject, const int & LinkInfoIndex, const Vector3 & GlobalPoint)
{
  // This function is used to calculate robot's self-collision information

  // Frame3D LinkLocalFrame = SimRobot.links[LinkIndex].T_World;
  // Vector3 ImpulseForceLocal = ImpulseForce;
  // LinkLocalFrame.mulPointInverse(ImpulseForce + LinkLocalFrame.t, ImpulseForceLocal);

}


static std::vector<cSpline3> cSplineGene(const Robot & SimRobot, SelfLinkGeoInfo & SelfLinkGeoObj, ReachabilityMap & RMObject, const int & LinkInfoIndex, const std::vector<Vector3> & Points, Vector3 & ShiftPoint, bool & FeasibleFlag)
{
  // Actually, the shift function has not been fully tested!
  Vec3f SplinePoints[Points.size()];
  for (int i = 0; i < Points.size(); i++)
  {
    Vec3f PointVec(Points[i].x, Points[i].y, Points[i].z);
    SplinePoints[i] = PointVec;
  }
  const int numPoints = sizeof(SplinePoints) / sizeof(SplinePoints[0]);
  cSpline3 splines[numPoints + 1];
  int numSplines = SplinesFromPoints(numPoints, SplinePoints, numPoints + 1, splines);
  std::vector<cSpline3> SplineObj;
  SplineObj.reserve(numSplines);

  std::vector<Vector3> ShiftPointVec;
  std::vector<double> ShiftPointDisVec;
  // std::vector<Vector3> TransitionPoints;
  const int GridNo = 10;
  float sUnit = 1.0/(1.0 * GridNo);
  for (int i = 0; i < numSplines; i++)
  {
    for (int j = 0; j < GridNo; j++)
    {
      float s = 1.0 * j * sUnit;
      Vec3f ps = Position (splines[i], s);
      Vector3 SplinePoint(ps.x, ps.y, ps.z);
      double SplinePointDis = NonlinearOptimizerInfo::SDFInfo.SignedDistance(SplinePoint);
      if(SplinePointDis<0)
      {
        ShiftPointVec.push_back(SplinePoint);
        ShiftPointDisVec.push_back(SplinePointDis);
      }
    }
  }

  switch (ShiftPointVec.size())
  {
    case 0:
    {
      FeasibleFlag = true;
      for (int i = 0; i < numSplines; i++)
      {
        SplineObj.push_back(splines[i]);
      }
    }
    break;
    default:
    {
      // Choose the point with the largest penetration.
      FeasibleFlag = false;
      int ShiftPointIndex = std::distance(ShiftPointDisVec.begin(), std::min_element(ShiftPointDisVec.begin(), ShiftPointDisVec.end()));
      ShiftPoint = ShiftPointVec[ShiftPointIndex];
    }
    break;
  }
  return SplineObj;
}

static std::vector<cSpline3> SplineObjGene(const Robot & SimRobot, SelfLinkGeoInfo & SelfLinkGeoObj, ReachabilityMap & RMObject, const int & LinkInfoIndex, const Vector3 & PosInit, const Vector3 & NormalInit, const Vector3 & PosGoal, const Vector3 & NormalGoal, bool & FeasiFlag)
{
  std::vector<Vector3> Points = BasePointsGene(PosInit, NormalInit, PosGoal, NormalGoal);   // Now we have the reference points!
  double SelfTol = SelfCollisionDist(SimRobot, SelfLinkGeoObj, RMObject, LinkInfoIndex, PosInit, PosGoal);

  FeasiFlag = false;
  const int TotalIter = 10;
  int CurrentIter = 0;
  std::vector<cSpline3> SplineObj;
  while((FeasiFlag == false)&&(CurrentIter<=TotalIter))
  {
    Vector3 ShiftPoint;
    SplineObj = cSplineGene(SimRobot, SelfLinkGeoObj, RMObject, LinkInfoIndex, Points, ShiftPoint, FeasiFlag);
    switch (FeasiFlag)
    {
      case true:
      break;
      default:
      {
        bool ShiftFlag;
        ShiftPoint = ShiftPointFunc(ShiftPoint, ShiftFlag);
        switch (ShiftFlag)
        {
          case false:
          {
            // If ShiftFlag turns out to be failure, then we need to terminate the whole loop
            return SplineObj;
          }
          break;
          default:
          break;
        }
        std::vector<Vector3> NewPoints(Points.size()+1);
        for (int i = 0; i < Points.size(); i++)
        {
          NewPoints[i] = Points[i];
        }
        NewPoints[Points.size()] = ShiftPoint;
        Points = NewPoints;
      }
      break;
    }
    CurrentIter++;
  }
  return SplineObj;
}

std::vector<cSpline3> TransientTrajGene(const Robot & SimRobot, const int & LinkInfoIndex, SelfLinkGeoInfo & SelfLinkGeoObj, const std::vector<LinkInfo> & RobotLinkInfo, const Vector3 & PosInit, const Vector3 & PosGoal, ReachabilityMap & RMObject, bool & TransFeasFlag)
{
  // This function is used to generate robot' tranistion trajecotries given initial configuration and final configuration.
  // A Hermite spline is constructed with the information of position and velocity.

  Vector3 NormalInit = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(PosInit);
  Vector3 NormalGoal = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(PosGoal);
  std::vector<cSpline3> SplineObj = SplineObjGene(SimRobot, SelfLinkGeoObj, RMObject, LinkInfoIndex, PosInit, NormalInit, PosGoal, NormalGoal, TransFeasFlag);

  const int SplineNumber = SplineObj.size();
  const int SplineGrid = 5;
  std::vector<Vector3> TransitionPoints(SplineNumber * SplineGrid + 1);

  double sUnit = 1.0/(1.0 * SplineGrid);
  int TransitionIndex = 0;
  for (int i = 0; i < SplineNumber; i++)
  {
    for (int j = 0; j < SplineGrid; j++)
    {
      double s = 1.0 * j * sUnit;
      Vec3f ps = Position (SplineObj[i], s);
      Vector3 SplinePoint(ps.x, ps.y, ps.z);
      TransitionPoints[TransitionIndex] = SplinePoint;
      TransitionIndex++;
    }
  }
  // The last waypoint
  Vec3f ps = Position (SplineObj[SplineNumber-1], 1.0);
  Vector3 SplinePoint(ps.x, ps.y, ps.z);
  TransitionPoints[TransitionIndex] = SplinePoint;
  Vector3Writer(TransitionPoints, "TransitionPoints");

  return SplineObj;

}
