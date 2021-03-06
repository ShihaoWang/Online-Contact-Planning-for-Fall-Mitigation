#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <ctime>
#include "Splines.h"

using namespace SplineLib;

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
  const int segmentNo = 5;
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

static double SelfCollisionDist(SelfLinkGeoInfo & SelfLinkGeoObj, const int & LinkInfoIndex, const Vector3 & PosInit, const Vector3 & PosGoal)
{
  // This function helps calculate robot's clearance for collision-avoidance
  double  InitDist, GoalDist;
  Vector3 InitGrad, GoalGrad;
  SelfLinkGeoObj.SelfCollisionDistNGrad(LinkInfoIndex, PosInit, InitDist, InitGrad);
  SelfLinkGeoObj.SelfCollisionDistNGrad(LinkInfoIndex, PosGoal, GoalDist, GoalGrad);

  return min(InitDist, GoalDist);
}

static std::vector<cSpline3> cSplineGene(const std::vector<Vector3> & Points, const int & LinkInfoIndex, const double & SelfTol, const SelfLinkGeoInfo & SelfLinkGeoObj, ReachabilityMap & RMObject, int & PointIndex, Vector3 & ShiftPoint, bool & FeasibleFlag)
{
  // Algorithm stops when tolerance cannot be satisfied!
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
  std::vector<int> SplineIndexVec;
  const int GridNo = 10;
  float sUnit = 1.0/(1.0 * GridNo);
  for (int i = 0; i < numSplines; i++)
  {
    for (int j = 0; j < GridNo-1; j++)
    {
      float s = 1.0 * j * sUnit;
      Vec3f ps = Position (splines[i], s);
      Vector3 SplinePoint(ps.x, ps.y, ps.z);
      double SplinePointDis = NonlinearOptimizerInfo::SDFInfo.SignedDistance(SplinePoint);
      if(SplinePointDis<0)
      {
        SplineIndexVec.push_back(i);
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
      PointIndex = SplineIndexVec[ShiftPointIndex];
      ShiftPoint = ShiftPointVec[ShiftPointIndex];
    }
    break;
  }
  return SplineObj;
}

static std::vector<Vector3> SpatialPointShifter(const std::vector<Vector3> & Points, const int & LinkInfoIndex, const double & SelfTol, SelfLinkGeoInfo & SelfLinkGeoObj, ReachabilityMap & RMObject, bool & ShiftFeasFlag)
{
  // This function is used to shift Points according to SelfTol
  // Here we only allow this shift to be conducted N times.
  const int TotalShiftTime = 10;
  std::vector<Vector3> NewPoints;
  NewPoints.push_back(Points[0]);

  int PointCount = 1;
  while (PointCount<Points.size()-1)
  {
    bool SelfShitFlag, EnviShitFlag;
    Vector3 CurPt = Points[PointCount];
    // Self-Collision Distance
    double  CurSelfPtDist;
    Vector3 CurSelfPtGrad;
    SelfLinkGeoObj.SelfCollisionDistNGrad(LinkInfoIndex, CurPt, CurSelfPtDist, CurSelfPtGrad);
    double CurEnvPtDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(CurPt);
    if((CurSelfPtDist<SelfTol)||(CurEnvPtDist<0))
    {
      // Then shift action is needed!
      Vector3 NewPt = CurPt;
      int ShiftTime = 0;
      double ShiftDistUnit = 0.025;     // 2.5cm for example
      while (ShiftTime<TotalShiftTime)
      {
        Vector3 SelfDirection(0.0, 0.0, 0.0);
        SelfShitFlag = false;
        if(CurSelfPtDist<SelfTol)
        {
          SelfDirection = CurSelfPtGrad;
          SelfShitFlag = true;
        }
        Vector3 EnviDirection(0.0, 0.0, 0.0);
        EnviShitFlag = false;
        if(CurEnvPtDist<0)
        {
          EnviDirection = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(NewPt);
          EnviShitFlag = true;
        }
        if((!SelfShitFlag)&&(!EnviShitFlag))
        {
          break;
        }
        Vector3 ShiftDirection = SelfDirection + EnviDirection;
        ShiftDirection.setNormalized(ShiftDirection);
        NewPt += ShiftDistUnit * ShiftDirection;
        SelfLinkGeoObj.SelfCollisionDistNGrad(LinkInfoIndex, NewPt, CurSelfPtDist, CurSelfPtGrad);
        CurEnvPtDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(CurPt);
        ShiftTime++;
      }
      if((!SelfShitFlag)&&(!EnviShitFlag))
      {
        NewPoints.push_back(NewPt);
      }
      else
      {
        ShiftFeasFlag = false;
        return NewPoints;
      }
    }
    else
    {
      NewPoints.push_back(CurPt);
    }
    PointCount++;
  }
  NewPoints.push_back(Points[Points.size()-1]);
  ShiftFeasFlag = true;
  return NewPoints;
}

static Vector3 SinglePointShifter(const Vector3 & Point, const int & LinkInfoIndex, const double & SelfTol, SelfLinkGeoInfo & SelfLinkGeoObj, ReachabilityMap & RMObject, bool & ShiftFeasFlag)
{
  double ShiftDistUnit = 0.025;     // 2.5cm for example
  Vector3 NewPoint = Point;
  const int TotalShiftTime = 10;
  int CurShiftTime = 0;

  bool SelfShitFlag, EnviShitFlag;
  double  CurSelfPtDist;
  Vector3 CurSelfPtGrad;
  SelfLinkGeoObj.SelfCollisionDistNGrad(LinkInfoIndex, Point, CurSelfPtDist, CurSelfPtGrad);
  double CurEnvPtDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(Point);
  ShiftFeasFlag = false;
  while(CurShiftTime<TotalShiftTime)
  {
    Vector3 SelfDirection(0.0, 0.0, 0.0);
    SelfShitFlag = false;
    if(CurSelfPtDist<SelfTol)
    {
      SelfDirection = CurSelfPtGrad;
      SelfShitFlag = true;
    }
    Vector3 EnviDirection(0.0, 0.0, 0.0);
    EnviShitFlag = false;
    if(CurEnvPtDist<0)
    {
      EnviDirection = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(NewPoint);
      EnviShitFlag = true;
    }
    if((!SelfShitFlag)&&(!EnviShitFlag))
    {
      break;
    }
    Vector3 ShiftDirection = SelfDirection + EnviDirection;
    ShiftDirection.setNormalized(ShiftDirection);
    NewPoint += ShiftDistUnit * ShiftDirection;
    SelfLinkGeoObj.SelfCollisionDistNGrad(LinkInfoIndex, NewPoint, CurSelfPtDist, CurSelfPtGrad);
    CurEnvPtDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(NewPoint);
    CurShiftTime++;
  }
  if((!SelfShitFlag)&&(!EnviShitFlag))
  {
    ShiftFeasFlag = true;
  }
  return NewPoint;
}

static std::vector<cSpline3> SplineObjGene(SelfLinkGeoInfo & SelfLinkGeoObj, ReachabilityMap & RMObject, const int & LinkInfoIndex, const Vector3 & PosInit, const Vector3 & NormalInit, const Vector3 & PosGoal, const Vector3 & NormalGoal, bool & FeasiFlag)
{
  // This function is used to generate a collision-free path!
  std::vector<cSpline3> SplineObj;
  std::vector<Vector3> Points = BasePointsGene(PosInit, NormalInit, PosGoal, NormalGoal);
  // Vector3Writer(Points, "TransitionPoints");
  double SelfTol = SelfCollisionDist(SelfLinkGeoObj, LinkInfoIndex, PosInit, PosGoal);
  bool InitShiftFeasFlag;     // For the shift of initial pts.
  Points = SpatialPointShifter(Points, LinkInfoIndex, SelfTol, SelfLinkGeoObj, RMObject, InitShiftFeasFlag);
  // Vector3Writer(Points, "TransitionPoints");
  FeasiFlag = false;
  if(!InitShiftFeasFlag)
  {
    return SplineObj;
  }

  // Then the task is to generate a path which is collision-free.
  const int TotalIter = 10;
  int CurrentIter = 0;
  while((FeasiFlag == false)&&(CurrentIter<=TotalIter))
  {
    Vector3 ShiftPoint;
    int ShiftPointIndex;
    SplineObj = cSplineGene(Points, LinkInfoIndex, SelfTol, SelfLinkGeoObj, RMObject, ShiftPointIndex, ShiftPoint, FeasiFlag);
    if(!FeasiFlag)
    {
      bool ShiftFlag;
      Vector3 NewShiftPoint = SinglePointShifter(ShiftPoint, LinkInfoIndex, SelfTol, SelfLinkGeoObj, RMObject, ShiftFlag);
      if(!ShiftFlag)
      {
        return SplineObj;
      }
      else
      {
        // Insert the NewShiftPoint back into Points vector
        std::vector<Vector3> NewPoints(Points.begin(), Points.begin() + ShiftPointIndex);
        NewPoints.push_back(NewShiftPoint);
        NewPoints.insert(NewPoints.end(), Points.begin() + ShiftPointIndex + 1, Points.end());
        Points = NewPoints;
      }
    }
    CurrentIter++;
  }
  return SplineObj;
}

std::vector<cSpline3> TransientTrajGene(const Robot & SimRobot, const int & LinkInfoIndex, SelfLinkGeoInfo & SelfLinkGeoObj, const std::vector<LinkInfo> & RobotLinkInfo, const Vector3 & PosInit, const Vector3 & PosGoal, ReachabilityMap & RMObject, DataRecorderInfo & DataRecorderObj, bool & TransFeasFlag)
{
  // This function is used to generate robot' tranistion trajecotries given initial configuration and final configuration.
  // A Hermite spline is constructed with the information of position and velocity.
  Vector3 NormalInit = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(PosInit);
  Vector3 NormalGoal = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(PosGoal);
  RobotLink3D Link_i = SimRobot.links[RobotLinkInfo[LinkInfoIndex].LinkIndex];

  // double GoalDist; Vector3 GoalGrad;
  // SelfLinkGeoObj.SelfCollisionDistNGrad(LinkInfoIndex, PosGoal, GoalDist, GoalGrad);

  Vector3 EnviDirection;
  double PosInitSelfCollisionDist;
  SelfLinkGeoObj.SingleLinkDistNGrad(LinkInfoIndex, PosInit, PosInitSelfCollisionDist, EnviDirection);

  Vector3 AlignDirection;
  AlignDirection.x = Link_i.T_World.R.data[2][0];
  AlignDirection.y = Link_i.T_World.R.data[2][1];
  AlignDirection.z = Link_i.T_World.R.data[2][2];

  Vector3 InitDir = PosGoal - PosInit;
  // NormalInit += AlignDirection + EnviDirection + InitDir;
  // NormalInit = EnviDirection;
  // NormalInit = InitDir + AlignDirection;
  NormalInit = InitDir;
  NormalInit.setNormalized(NormalInit);
  // NormalGoal = -GoalGrad;

  std::vector<cSpline3> SplineObj = SplineObjGene(SelfLinkGeoObj, RMObject, LinkInfoIndex, PosInit, NormalInit, PosGoal, NormalGoal, TransFeasFlag);

  if(TransFeasFlag)
  {
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
    DataRecorderObj.TransitionPoints = TransitionPoints;
    // Vector3Writer(TransitionPoints, "TransitionPoints");
  }
  return SplineObj;
}
