#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include <omp.h>
#include <iterator>
#include <map>
#include <random>

// Here this function is used to generate a sufficiently dense reachability map for end effector(s)
ReachabilityMap ReachabilityMapGenerator(Robot & SimRobot, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<int> & TorsoLink)
{
  // This function is used to generate the reachability map for end effector.
  // Due to the fact that the area of sphere is propotional to r^2, the number of the sampled data is propotional to r^2 on each sphere.
  double MaxRadius = 0.7;
  int LayerNumber = 70;
  int PointNumberOnInner = 25;
  double LayerDiff = MaxRadius/(LayerNumber * 1.0);
  double MinRadius = LayerDiff;

  // Three uniform distributions
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> Xdis(-MaxRadius, MaxRadius);
  std::uniform_real_distribution<> Ydis(-MaxRadius, MaxRadius);
  std::uniform_real_distribution<> Zdis(-MaxRadius, MaxRadius);

  std::map<int, std::vector<RMPoint>> RMObj;

  int TotalPoint = 0;
  for (int i = 0; i < LayerNumber; i++)
  {
    double Radius = (1.0 * i + 1.0) * LayerDiff;
    int LayerPointLimit = (i + 1) * (i + 1) * PointNumberOnInner;
    int LayerPointNumber = 0;
    std::vector<RMPoint> RMLayer;
    RMLayer.reserve(LayerPointLimit);
    while (LayerPointNumber<LayerPointLimit)
    {
      double PointX = Xdis(gen);
      double PointY = Ydis(gen);
      double PointZ = Zdis(gen);
      double PointNorm = std::sqrt(PointX * PointX + PointY * PointY + PointZ * PointZ);
      double Ratio = PointNorm/Radius;
      Vector3 RMPointPos(PointX/Ratio, PointY/Ratio, PointZ/Ratio);
      RMPoint RMPoint_i(Radius, RMPointPos);
      RMLayer.push_back(RMPoint_i);
      LayerPointNumber++;
      TotalPoint++;
    }
    RMObj[i] = RMLayer;
  }
  ReachabilityMap RMObject(RMObj);
  RMObject.ReachabilityMapPara(MaxRadius, LayerNumber, PointNumberOnInner, LayerDiff, MinRadius);

  // The next step is to estimate the end effector radius.
  int DOF = SimRobot.q.size();
  std::vector<double> ZeroConfiguration(DOF);
  for (int i = 0; i < DOF; i++)
  {
    ZeroConfiguration[i] = 0.0;
  }
  SimRobot.UpdateConfig(Config(ZeroConfiguration));
  std::vector<double> EndEffectorRadius(RobotLinkInfo.size());

  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    int ParentIndex = -1;
    int CurrentIndex = RobotLinkInfo[i].LinkIndex;
    while(std::find(TorsoLink.begin(), TorsoLink.end(), ParentIndex)==TorsoLink.end())
    {
      ParentIndex = SimRobot.parents[CurrentIndex];
      CurrentIndex = ParentIndex;
    }
    Vector3 PivotalPos, EndPos, PivotalRef(0.0, 0.0, 0.0);
    SimRobot.GetWorldPosition(PivotalRef, ParentIndex, PivotalPos);
    SimRobot.GetWorldPosition(RobotLinkInfo[i].AvgLocalContact, RobotLinkInfo[i].LinkIndex, EndPos);

    Vector3 Pivotal2End = PivotalPos - EndPos;
    double Pivotal2EndRadius = sqrt(Pivotal2End.x * Pivotal2End.x + Pivotal2End.y * Pivotal2End.y + Pivotal2End.z * Pivotal2End.z);
    EndEffectorRadius[i] = Pivotal2EndRadius;
  }
  RMObject.EndEffectorRadius = EndEffectorRadius;
  return RMObject;
}
