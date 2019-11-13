#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include <omp.h>
#include <iterator>
#include <map>
#include <random>

// Here this function is used to generate a sufficiently dense reachability map for end effector(s)
std::map<int, std::vector<RMPoint>> ReachabilityMapGenerator()
{
  // This function is used to generate the reachability map for end effector.
  // Due to the fact that the area of sphere is propotional to r^2.
  double MaxRadius = 1.0;
  int LayerNumber = 100;
  int PointNumberOnInner = 25;
  double LayerDiff = MaxRadius/(LayerNumber * 1.0);

  // Three uniform distributions
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> Xdis(-MaxRadius, MaxRadius);
  std::uniform_real_distribution<> Ydis(-MaxRadius, MaxRadius);
  std::uniform_real_distribution<> Zdis(-MaxRadius, MaxRadius);

  std::map<int, std::vector<RMPoint>> Obj;

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
    }
    Obj[i] = RMLayer;
    int a = 1;
  }
}
