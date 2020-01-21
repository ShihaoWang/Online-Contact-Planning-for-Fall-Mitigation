#include "RobotInfo.h"
#include "CommonHeader.h"
#include <set>

#include <cilantro/convex_polytope.hpp>
#include <cilantro/point_cloud.hpp>
#include <cilantro/visualizer.hpp>
#include <cilantro/common_renderables.hpp>
#include <cilantro/flat_convex_hull_3d.hpp>

// This function is used to calculate the reactive mass pendulum related functions
static double eps_dist = 0.001;        // 1 mm

std::vector<Vector3> ProjActContactPosGene(const std::vector<Vector3> & ActContactPositions)
{
  // Projection to ground
  std::vector<Vector3> ProjActContactPositions;
  ProjActContactPositions.reserve(ActContactPositions.size());
  double LowestHeight = 1000.0;
  for (int j = 0; j < ActContactPositions.size(); j++)
  {
    if(LowestHeight>ActContactPositions[j].z)
    {
      LowestHeight = ActContactPositions[j].z;
    }
  }
  for (int j = 0; j < ActContactPositions.size(); j++)
  {
    Vector3 ProjActContact(ActContactPositions[j].x, ActContactPositions[j].y, LowestHeight);
    ProjActContactPositions.push_back(ProjActContact);
  }
  return ProjActContactPositions;
}

PIPInfo PIPGeneratorInner(const Vector3& EdgeA, const Vector3& EdgeB, const Vector3& COM, const Vector3& COMVel)
{
  // Here the inputs are EdgeA and EdgeB, which should be chosen to be on the positive dirction of certain edge.
  // Now it is the CoM projection and basically the job is to find the orthogonal intersection point
  Vector3 EdgeA2COM = COM - EdgeA;
  Vector3 EdgeA2B = EdgeB - EdgeA;                      // x
  Vector3 x_unit, y_unit, z_unit;
  EdgeA2B.getNormalized(x_unit);
  double t = EdgeA2COM.dot(x_unit);
  Vector3 COMOnEdge = EdgeA + t * x_unit;
  Vector3 COMOnEdge2COM = COM - COMOnEdge;              // y
  COMOnEdge2COM.getNormalized(y_unit);
  z_unit = -1.0*cross(x_unit, y_unit);                  // z

  double L = COMOnEdge2COM.norm();
  double Ldot = COMVel.dot(y_unit);
  double thetadot = COMVel.dot(z_unit)/L;

  Vector3 g(0.0, 0.0, -9.81);
  Vector3 EdgeA2BProj(EdgeB.x - EdgeA.x, EdgeB.y - EdgeA.y, 0.0);
  Vector3 z_prime_unit = cross(EdgeA2BProj, -1.0 * g);
  z_prime_unit = z_prime_unit/z_prime_unit.norm();
  Vector3 y_prime_unit = cross(z_prime_unit, x_unit);
  double theta = atan2(-1.0 * COMOnEdge2COM.dot(z_prime_unit), COMOnEdge2COM.dot(y_prime_unit));
  double g_proj = -g.dot(y_prime_unit);

  double VertY = EdgeB.z - EdgeA.z;
  double VertX = (EdgeB.x - EdgeA.x)*(EdgeB.x - EdgeA.x) + (EdgeB.y - EdgeA.y) * (EdgeB.y - EdgeA.y);
  VertX = sqrt(VertX);
  double g_angle = abs(atan2(VertY, VertX))/3.1415926535 * 180.0;       // Expressed in degrees

  PIPInfo IPM_i(L, Ldot, theta, thetadot, g_proj, g_angle);

  IPM_i.x_unit = x_unit;
  IPM_i.y_unit = y_unit;
  IPM_i.z_unit = z_unit;     // Here x-y-z do not obey right hand rule.

  IPM_i.x_prime_unit = x_unit;
  IPM_i.y_prime_unit = y_prime_unit;
  IPM_i.z_prime_unit = z_prime_unit;

  IPM_i.EdgeA = EdgeA;
  IPM_i.EdgeB = EdgeB;
  IPM_i.Intersection = COMOnEdge;

  return IPM_i;
}

static Vector3 NormalFromThreePoints(const Vector3& Pt1, const Vector3& Pt2, const Vector3& Pt3)
{
  // This function is used to calculate the plane normal based on three points.
  // The sequence of TriangleMesh follows the counterclockwise direction
  Vector3 Pt1to2 = Pt2 - Pt1;
  Vector3 Pt2to3 = Pt3 - Pt2;

  Vector3 TriNormal; TriNormal.setCross(Pt1to2, Pt2to3);

  double TriNormalMagnitude = sqrt(TriNormal.x * TriNormal.x + TriNormal.y * TriNormal.y + TriNormal.z * TriNormal.z);

  TriNormal.x = TriNormal.x/TriNormalMagnitude;
  TriNormal.y = TriNormal.y/TriNormalMagnitude;
  TriNormal.z = TriNormal.z/TriNormalMagnitude;

  return TriNormal;
}

static int PointsCoplanarEval(const std::vector<Vector3> & _CPVertices)
{
  // This function is used to evaluate whether the given data points are on the same plane
  int CoplanarFlag = 1;
  switch (_CPVertices.size()) {
    case 2:
    {
      return 1;
    }
    break;
    case 3:
    {
      return 1;
    }
    break;
    default:    // In this case, there are at least three data points so we have to test whether the given points are on the same plane or not
    {
      Vector3 PlaneNormal = NormalFromThreePoints(_CPVertices[0], _CPVertices[1],  _CPVertices[2]);
      Vector3 Vertex2Point;
      for (int i = 0; i < _CPVertices.size()-3; i++)
      {
        Vertex2Point = _CPVertices[i + 3] - _CPVertices[0];
        if (abs(PlaneNormal.dot(Vertex2Point))>eps_dist)
        {
          CoplanarFlag = 0;
          break;
        }
      }
    }
    break;
  }
  return CoplanarFlag;
}

int CollinearTest(const std::vector<Vector3> & _CPVertices)
{
  int Res = 0;
  switch (_CPVertices.size())
  {
    case 0:
    {
      return 1;
    }
    break;
    case 1:
    {
      return 1;
    }
    break;
    case 2:
    {
      return 1;
    }
    break;
    default:
    break;
  }
  std::vector<Eigen::Vector3f> Vertices;
  Vertices.reserve(_CPVertices.size());
  for (int i = 0; i < _CPVertices.size(); i++)
  {
    Vertices.emplace_back(_CPVertices[i].x,_CPVertices[i].y,_CPVertices[i].z);
  }

  cilantro::PointCloud3f datacloud(Vertices);

  // cilantro::FlatConvexHull3f flat_hull(datacloud.points, true, false);
  cilantro::FlatConvexHull3f flat_hull(datacloud.points, true, false);

  datacloud.points = flat_hull.reconstruct<2>(flat_hull.project<2>(datacloud.points));
  const auto& face_v_ind = flat_hull.getFacetVertexIndices();
  const int SegmentNumber = face_v_ind.size();

  if(SegmentNumber<3)
  {
    // For a face to exist, there has to be greater than or equal to 3
    Res = 1;
  }
  else
  {
    Res = 0;
  }
  return Res;
}

FacetInfo FlatContactHullGeneration(const std::vector<Vector3> & _CPVertices, int & FacetFlag)
{
  FacetInfo FacetInfoObj;
  std::vector<Eigen::Vector3f> Vertices;
  Vertices.reserve(_CPVertices.size());
  for (int i = 0; i < _CPVertices.size(); i++)
  {
    Vertices.emplace_back(_CPVertices[i].x,_CPVertices[i].y,_CPVertices[i].z);
  }

  cilantro::PointCloud3f datacloud(Vertices);

  // cilantro::FlatConvexHull3f flat_hull(datacloud.points, true, false);
  cilantro::FlatConvexHull3f flat_hull(datacloud.points, true, false);

  datacloud.points = flat_hull.reconstruct<2>(flat_hull.project<2>(datacloud.points));
  const auto& face_v_ind = flat_hull.getFacetVertexIndices();
  const int SegmentNumber = face_v_ind.size();
  cilantro::VectorSet3f src_points(3, SegmentNumber);
  cilantro::VectorSet3f dst_points(3, SegmentNumber);

  switch (face_v_ind.size())
  {
    case 0:
    {
      // This means that the current FacetInfo is invalid.
      FacetFlag = 0;
      return FacetInfoObj;
    }
    break;
    default:
    {
      FacetFlag = 1;
    }
    break;
  }

  // src_points/dst_points will always ouputs convex hull edges but these edges may not be in the counterclockwise order.
  Eigen::Vector3f FirstA = flat_hull.getVertices3D().col(face_v_ind[0][0]);
  Eigen::Vector3f FirstB = flat_hull.getVertices3D().col(face_v_ind[0][1]);

  int ReverseOrder = 0;

  Vector3 NormalVec(0.0, 0.0, 1.0);     // Initialize it to be a unit vector
  int NormalVecFlag = 0;

  for (size_t i = 1; i < SegmentNumber; i++)
  {
    Eigen::Vector3f SecondA = flat_hull.getVertices3D().col(face_v_ind[i][0]);
    Eigen::Vector3f SAmFB = SecondA - FirstB;
    if(SAmFB.norm()<eps_dist)
    {
      Eigen::Vector3f SecondB = flat_hull.getVertices3D().col(face_v_ind[i][1]);
      Eigen::Vector3f FirstEdge = FirstB - FirstA;
      Eigen::Vector3f SecondEdge = SecondB - SecondA;

      Eigen::Vector3f EdgeNorm = FirstEdge.cross(SecondEdge);
      float NormProj = EdgeNorm(2);
      if(NormProj<0)
      {
        ReverseOrder = 1;
        Vector3 FaceNorm(-1.0 * EdgeNorm(0), -1.0 * EdgeNorm(1), -1.0 * EdgeNorm(2));
        NormalVec.setNormalized(FaceNorm);
        break;
      }
      else
      {
        switch (NormalVecFlag)
        {
          case 0:
          Vector3 FaceNorm(EdgeNorm(0), EdgeNorm(1), EdgeNorm(2));
          NormalVec.setNormalized(FaceNorm);
          NormalVecFlag = 1;
          break;
        }
      }
    }
  }

  FacetInfoObj.FacetNormUpdate(NormalVec);
  FacetInfoObj.FacetEdges.reserve(SegmentNumber);
  FacetInfoObj.EdgeNorms.reserve(SegmentNumber);
  for (int i = 0; i < SegmentNumber; i++)
  {
    Eigen::Vector3f FirstA = flat_hull.getVertices3D().col(face_v_ind[i][0]);
    Eigen::Vector3f FirstB = flat_hull.getVertices3D().col(face_v_ind[i][1]);
    switch (ReverseOrder)
    {
      case 0:
      {     // There is no need to change src_point/dst_point order
            Vector3 EdgeFront(FirstA(0), FirstA(1), FirstA(2));
            Vector3 EdgeBack(FirstB(0), FirstB(1), FirstB(2));
            FacetInfoObj.FacetEdges.push_back(make_pair(EdgeFront, EdgeBack));

            Vector3 EdgeFront2Back = EdgeBack - EdgeFront;
            Vector3 EdgeNorm_i = cross(FacetInfoObj.FacetNorm, EdgeFront2Back);
            FacetInfoObj.EdgeNorms.push_back(EdgeNorm_i/EdgeNorm_i.norm());
      }
      break;
      case 1:
      {
            // src_point/dst_point order have to be switched.
            // There is no need to change src_point/dst_point order
            Vector3 EdgeFront(FirstB(0), FirstB(1), FirstB(2));
            Vector3 EdgeBack(FirstA(0), FirstA(1), FirstA(2));
            FacetInfoObj.FacetEdges.push_back(make_pair(EdgeFront, EdgeBack));

            Vector3 EdgeFront2Back = EdgeBack - EdgeFront;
            Vector3 EdgeNorm_i = cross(FacetInfoObj.FacetNorm, EdgeFront2Back);
            FacetInfoObj.EdgeNorms.push_back(EdgeNorm_i/EdgeNorm_i.norm());
      }
      break;
      default:
      break;
    }
  }
  return FacetInfoObj;
}

static void ReplicationReduction(std::vector<Vector3> & _EdgeAVec, std::vector<Vector3> & _EdgeBVec, const Vector3 & _A, const Vector3 & _B)
{
  // This function is used to get rid of the replicative term and update the new EdgeA and EdgeB vector
  double eps = 1e-10;
  for (int i = 0; i < _EdgeAVec.size(); i++)
  {
    Vector3 DiffA = _EdgeAVec[i] - _A;
    Vector3 DiffB = _EdgeBVec[i] - _B;
    double DiffAVal = DiffA.dot(DiffA);
    double DiffBVal = DiffB.dot(DiffB);
    if((DiffAVal <= eps)&&(DiffBVal <= eps))
    {
      // If this loop can reach here, this means that there is no overlapping of these indices.
      return;
    }
    DiffA = _EdgeAVec[i] - _B;
    DiffB = _EdgeBVec[i] - _A;
    DiffAVal = DiffA.dot(DiffA);
    DiffBVal = DiffB.dot(DiffB);
    if((DiffAVal <= eps)&&(DiffBVal <= eps))
    {
      // If this loop can reach here, this means that there is no overlapping of these indices.
      return;
    }
  }
  _EdgeAVec.push_back(_A);
  _EdgeBVec.push_back(_B);
}

std::vector<FacetInfo> NonflatContactHullGeneration(const std::vector<Vector3> & _CPVertices, std::vector<Vector3> & _CPVertex ,std::vector<Vector3> & _CPEdgeA, std::vector<Vector3> & _CPEdgeB)
{
  std::vector<Eigen::Vector3f> Vertices;
  Vertices.reserve(_CPVertices.size());
  for (int i = 0; i < _CPVertices.size(); i++)
  {
    Vertices.emplace_back(_CPVertices[i].x,_CPVertices[i].y,_CPVertices[i].z);
  }
  cilantro::PointCloud3f datacloud(Vertices);
  double CH_eps_dist = 1e-10;
  // cilantro::ConvexHull3f convex_hull(datacloud.points, true, false, CH_eps_dist);
  cilantro::ConvexHull3f convex_hull(datacloud.points, true, false, CH_eps_dist);


  const int VertexNumber = convex_hull.getVertexPointIndices().size();
  const int FacetNumber = convex_hull.getFacetVertexIndices().size();

  // RealVertices.reserve(VertexNumber);
  // std::vector<int> VertexIndices = convex_hull.getVertexPointIndices();

  std::vector<Vector3> RealVertices;
  RealVertices.reserve(VertexNumber);

  for (int i = 0; i < VertexNumber; i++)
  {
    // This part is used to take out the convex vertices from convex hull.
    Vector3 Vertex_i(convex_hull.getVertices()(0, i), convex_hull.getVertices()(1, i), convex_hull.getVertices()(2, i));
    RealVertices.push_back(Vertex_i);
  }
  _CPVertex = RealVertices;

  std::vector<Vector3> EdgeA, EdgeB;      // These are the vectors saving Edge indices.
  std::vector<FacetInfo> FacetInfoTotal(FacetNumber);
  for (int i = 0; i < FacetNumber; i++)
  {
    int Face_i_Vertices_Number = convex_hull.getFacetVertexIndices()[i].size();
    std::vector<Vector3> Face_i_Vertices;
    Face_i_Vertices.reserve(Face_i_Vertices_Number);
    for (int j = 0; j < Face_i_Vertices_Number; j++)
    {
      int Face_i_Vertex_j_Index = convex_hull.getFacetVertexIndices()[i][j];
      Vector3 Face_i_Vertex_j(convex_hull.getVertices()(0,Face_i_Vertex_j_Index), convex_hull.getVertices()(1,Face_i_Vertex_j_Index), convex_hull.getVertices()(2,Face_i_Vertex_j_Index));
      Face_i_Vertices.push_back(Face_i_Vertex_j);
    }
    int FacetFlag = 0;
    FacetInfo FaceInfo_j = FlatContactHullGeneration(Face_i_Vertices, FacetFlag);
    FaceInfo_j.EdgesUpdate();
    for (int j = 0; j < FaceInfo_j.FacetEdges.size(); j++)
    {
      Vector3 Face_i_Vertex_j = FaceInfo_j.FacetEdges[j].first;
      Vector3 Face_i_Vertex_jp1 = FaceInfo_j.FacetEdges[j].second;
      EdgeA.push_back(Face_i_Vertex_j);
      EdgeB.push_back(Face_i_Vertex_jp1);
    }
    FacetInfoTotal[i] = FaceInfo_j;
  }

  // Let's get rid of the same terms
  std::vector<Vector3> RealEdgeA, RealEdgeB;
  RealEdgeA.push_back(EdgeA[0]);
  RealEdgeB.push_back(EdgeB[0]);
  for (int i = 0; i < EdgeA.size()-1; i++)
  {
    // This function is used to select the non-replicative terms out from EdgeA and EdgeB.
    ReplicationReduction(RealEdgeA, RealEdgeB, EdgeA[i+1], EdgeB[i+1]);
  }
  // Give the value back into CPEdges
  _CPEdgeA = RealEdgeA;
  _CPEdgeB = RealEdgeB;

  return FacetInfoTotal;
}

std::vector<FacetInfo> ContactHullGeneration(const std::vector<Vector3>& _CPVertices, std::vector<Vector3> & CPVertex, std::vector<Vector3> & CPEdgeA, std::vector<Vector3> & CPEdgeB)
{
  // This function is used to generate the convex hull given the vertices
  int CoplanarFlag = PointsCoplanarEval(_CPVertices);
  std::vector<std::vector<Vector3>> EdgeVertices;
  switch (CoplanarFlag)
  {
    case 1:
    {
      // All the data points are on the same plane, so we should use the FlatConvexHull3f
      int FacetFlag = 0;
      FacetInfo FlatConvexHullObj= FlatContactHullGeneration(_CPVertices, FacetFlag);
      FlatConvexHullObj.EdgesUpdate();
      std::vector<FacetInfo> ConvexHullObj;
      ConvexHullObj.reserve(1);
      ConvexHullObj.push_back(FlatConvexHullObj);

      // Here CPVertex, CPEdgeA, CPEdgeB are to be updated!
      // CPVertex = FlatConvexHullObj.

      CPEdgeA.reserve(FlatConvexHullObj.FacetEdges.size());
      CPEdgeB.reserve(FlatConvexHullObj.FacetEdges.size());
      for (int i = 0; i < FlatConvexHullObj.FacetEdges.size(); i++)
      {
        CPEdgeA.push_back(FlatConvexHullObj.FacetEdges[i].first);
        CPEdgeB.push_back(FlatConvexHullObj.FacetEdges[i].second);
      }

      // On 2D plane, these two are the same.
      CPVertex = CPEdgeA;
      return ConvexHullObj;
    }
    break;
    case 0:
    {
      return NonflatContactHullGeneration(_CPVertices, CPVertex, CPEdgeA, CPEdgeB);
    }
    break;
    default:
    break;
  }
}

static double ZCoordinate(const double & _X, const double & _Y, const std::vector<Vector3> & CPVertices, int & PtIndex)
{
  // This function is used to select out the ZCoodrinate given _X and _Y.
  int CPNumber = CPVertices.size();
  std::vector<double> CPDiff(CPNumber);
  for (int i = 0; i < CPNumber; i++)
  {
    double Difference_i = (CPVertices[i].x - _X) * (CPVertices[i].x - _X) + (CPVertices[i].y - _Y) * (CPVertices[i].y - _Y);
    CPDiff[i] = Difference_i;
  }

  // The CPDiff
  int CPDiffIndex = std::distance(CPDiff.begin(), std::min_element(CPDiff.begin(), CPDiff.end()));
  return CPVertices[CPDiffIndex].z;
}

std::vector<PIPInfo> PIPGenerator(const std::vector<Vector3> & CPVertices, const Vector3 & COMPos, const Vector3 & COMVel)
{
  std::vector<PIPInfo> PIPTotal;
  std::vector<Vector3> SPVertices;
  SPVertices.reserve(CPVertices.size());
  for (int i = 0; i < CPVertices.size(); i++)
  {
    Vector3 SPVertex(CPVertices[i].x, CPVertices[i].y, 0.0);
    SPVertices.push_back(SPVertex);
  }
  // Now all the points have already been projected into a 2D plane
  int FacetFlag = 0;
  FacetInfo FlatConvexHullObj= FlatContactHullGeneration(SPVertices, FacetFlag);
  for (int i = 0; i < FlatConvexHullObj.FacetEdges.size(); i++)
  {
    int FirstPtIndex, SecondPtIndex;
    double Edge_i_First_z =   ZCoordinate(FlatConvexHullObj.FacetEdges[i].first.x,  FlatConvexHullObj.FacetEdges[i].first.y,    CPVertices, FirstPtIndex);
    double Edge_i_Second_z =  ZCoordinate(FlatConvexHullObj.FacetEdges[i].second.x, FlatConvexHullObj.FacetEdges[i].second.y,   CPVertices, SecondPtIndex);
    Vector3 FirstEdge(  FlatConvexHullObj.FacetEdges[i].first.x,   FlatConvexHullObj.FacetEdges[i].first.y,  Edge_i_First_z);
    Vector3 SecondEdge( FlatConvexHullObj.FacetEdges[i].second.x,  FlatConvexHullObj.FacetEdges[i].second.y, Edge_i_Second_z);
    PIPInfo PIPInfo_i = PIPGeneratorInner(FirstEdge, SecondEdge, COMPos, COMVel);
    if(PIPInfo_i.g_angle<80)
    {
      PIPTotal.push_back(PIPInfo_i);
    }
  }
  return PIPTotal;
}

void ConeUnitGenerator(const std::vector<Vector3> & ActContacts, SignedDistanceFieldInfo& SDFInfo, std::vector<Vector3> & ConeShiftedUnits, std::vector<Vector3> & ConeUnits, const int & edge_no, const double & mu)
{
  // Now let's compute the unit direction vector for vector approximation.
  Vector3 RefUnit;
  RefUnit = ActContacts[1] - ActContacts[2];
  RefUnit.setNormalized(RefUnit);

  ConeShiftedUnits.reserve(edge_no * ActContacts.size());
  ConeUnits.reserve(edge_no * ActContacts.size());
  for (int i = 0; i < ActContacts.size(); i++)
  {
    Vector3 ContactNormals_i = SDFInfo.SignedDistanceNormal(ActContacts[i]);
    ContactNormals_i.setNormalized(ContactNormals_i);
    Vector3 TangVector_i = cross(ContactNormals_i, RefUnit);
    TangVector_i.x = mu * TangVector_i.x;
    TangVector_i.y = mu * TangVector_i.y;
    TangVector_i.z = mu * TangVector_i.z;
    double RotAngleUnit = 2.0 * 3.1415926535/(edge_no * 1.0);
    AngleAxisRotation RotMatrix(RotAngleUnit, ContactNormals_i);
    Vector3 RefPoint(ContactNormals_i.x + TangVector_i.x, ContactNormals_i.y + TangVector_i.y, ContactNormals_i.z + TangVector_i.z);
    for (int j = 0; j < edge_no; j++)
    {
      Vector3 ConeShiftedUnit(RefPoint.x + ActContacts[i].x, RefPoint.y + ActContacts[i].y, RefPoint.z + ActContacts[i].z);
      ConeShiftedUnits.push_back(ConeShiftedUnit);
      Vector3 ConeUnit_i = ConeShiftedUnit - ActContacts[i];
      ConeUnits.push_back(ConeUnit_i);
      Vector3 RefPoint_new;
      RotMatrix.transformPoint(RefPoint, RefPoint_new);
      RefPoint = RefPoint_new;
    }
  }
}

void ContactPolytopeWriter(const std::vector<PIPInfo> & PIPTotal, const std::vector<string> & EdgeFileNames)
{
  std::ofstream fEdgeA;         fEdgeA.open(EdgeFileNames[0].c_str(), std::ios_base::app);
  std::ofstream fEdgeB;         fEdgeB.open(EdgeFileNames[1].c_str(), std::ios_base::app);
  std::ofstream fEdgeCOM;       fEdgeCOM.open(EdgeFileNames[2].c_str(), std::ios_base::app);
  std::ofstream fEdgex;         fEdgex.open(EdgeFileNames[3].c_str(), std::ios_base::app);
  std::ofstream fEdgey;         fEdgey.open(EdgeFileNames[4].c_str(), std::ios_base::app);
  std::ofstream fEdgez;         fEdgez.open(EdgeFileNames[5].c_str(), std::ios_base::app);

  for (int i = 0; i < PIPTotal.size(); i++)
  {
    fEdgeA<<std::to_string(PIPTotal[i].EdgeA.x)<<" "<<std::to_string(PIPTotal[i].EdgeA.y)<<" "<<std::to_string(PIPTotal[i].EdgeA.z)<<" ";
    fEdgeB<<std::to_string(PIPTotal[i].EdgeB.x)<<" "<<std::to_string(PIPTotal[i].EdgeB.y)<<" "<<std::to_string(PIPTotal[i].EdgeB.z)<<" ";
    fEdgeCOM<<std::to_string(PIPTotal[i].Intersection.x)<<" "<<std::to_string(PIPTotal[i].Intersection.y)<<" "<<std::to_string(PIPTotal[i].Intersection.z)<<" ";
    fEdgex<<std::to_string(PIPTotal[i].x_prime_unit.x)<<" "<<std::to_string(PIPTotal[i].x_prime_unit.y)<<" "<<std::to_string(PIPTotal[i].x_prime_unit.z)<<" ";
    fEdgey<<std::to_string(PIPTotal[i].y_prime_unit.x)<<" "<<std::to_string(PIPTotal[i].y_prime_unit.y)<<" "<<std::to_string(PIPTotal[i].y_prime_unit.z)<<" ";
    fEdgez<<std::to_string(PIPTotal[i].z_prime_unit.x)<<" "<<std::to_string(PIPTotal[i].z_prime_unit.y)<<" "<<std::to_string(PIPTotal[i].z_prime_unit.z)<<" ";
  }

  fEdgeA<<"\n";                   fEdgeA.close();
  fEdgeB<<"\n";                   fEdgeB.close();
  fEdgeCOM<<"\n";                 fEdgeCOM.close();
  fEdgex<<"\n";                   fEdgex.close();
  fEdgey<<"\n";                   fEdgey.close();
  fEdgez<<"\n";                   fEdgez.close();
  return;
}

double RBGenerator(const std::vector<PIPInfo> & PIPTotal, int & PIPIndex)
{
  std::vector<double> RBFailureMetric(PIPTotal.size());
  for (int i = 0; i < PIPTotal.size(); i++)
  {
    // For each edge, the comparison is based on the instantaneus kinetic energy and the maximum potential energy can be accumulated.
    double theta = PIPTotal[i].theta;
    double thetadot = PIPTotal[i].thetadot;
    double L = PIPTotal[i].L;
    double Ldot = PIPTotal[i].Ldot;
    double g = PIPTotal[i].g;

    if(theta>0.0)
    {
      if(thetadot>0.0)
      {
        RBFailureMetric[i] = 0.0;
      }
      else
      {
        // This means that the pendulum is tilting to the negative direction.
        double KE = 0.5 * L * L * thetadot * thetadot;
        double PE = g * L * (1.0 - cos(theta));
        RBFailureMetric[i] = KE - PE;
      }
    }
    else
    {
      // This means that the current robot is at the negative side.
      double KE = 0.5 * L * L * thetadot * thetadot;
      double PE = g * L * (1.0 - cos(theta));
      if(thetadot>0.0)
      {
        RBFailureMetric[i] = PE - KE;
      }
      else
      {
        RBFailureMetric[i] = KE;
      }
    }
  }
  double FailureMetricRB = *max_element(RBFailureMetric.begin(), RBFailureMetric.end());
  FailureMetricRB = max(FailureMetricRB, 0.0);
  if(FailureMetricRB>0)
  {
    // This indicates that robot's is going to have failure
    PIPIndex = std::distance(RBFailureMetric.begin(), std::max_element(RBFailureMetric.begin(), RBFailureMetric.end()));
  }
  else
  {
    PIPIndex = -1;
  }
  return FailureMetricRB;
}

int PIPIndexFinder(const std::vector<PIPInfo> & PIPTotal, const Vector3 & RefPos)
{
  std::vector<double> CP_Pos(PIPTotal.size());
  std::vector<double> RefPosCMP(PIPTotal.size());
  for (int i = 0; i < PIPTotal.size(); i++)
  {
    double L = PIPTotal[i].L;
    double theta = PIPTotal[i].theta;
    double thetadot = PIPTotal[i].thetadot;
    double g = PIPTotal[i].g;

    double CP_x = L * sin(theta);
    double CP_xdot = L * thetadot * cos(theta);
    double CP_L = L * cos(theta);

    if(CP_L<=0)
    {
      CP_Pos[i] = 0.0;
    }
    else
    {
      double CP_g = g;
      double CP_xbar = CP_x + CP_xdot/sqrt(CP_g/CP_L);
      CP_Pos[i] = CP_xbar;
    }

    Vector3 EdgeA2Ref = RefPos - PIPTotal[i].EdgeA;
    Vector3 EdgeA2B = PIPTotal[i].EdgeB - PIPTotal[i].EdgeA;                      // x
    double t = EdgeA2Ref.dot(PIPTotal[i].x_unit);
    Vector3 COMOnEdge = PIPTotal[i].EdgeA + t * PIPTotal[i].x_unit;
    Vector3 COMOnEdge2COM = RefPos - COMOnEdge;              // y
    double RefPosCMPVal = COMOnEdge2COM.norm();
    if(CP_Pos[i]>0)
    {
      RefPosCMP[i] = RefPosCMPVal;
    }
    else
    {
      RefPosCMP[i] = -RefPosCMPVal;
    }
  }
  int PIPIndex = std::distance(RefPosCMP.begin(), std::min_element(RefPosCMP.begin(), RefPosCMP.end()));
  return PIPIndex;
}

double CapturePointGenerator(const std::vector<PIPInfo> & PIPTotal, int & PIPIndex)
{
  std::vector<double> CP_Pos(PIPTotal.size());
  for (int i = 0; i < PIPTotal.size(); i++)
  {
    double L = PIPTotal[i].L;
    double theta = PIPTotal[i].theta;
    double thetadot = PIPTotal[i].thetadot;
    double g = PIPTotal[i].g;

    double CP_x = L * sin(theta);
    double CP_xdot = L * thetadot * cos(theta);
    double CP_L = L * cos(theta);

    if(CP_L<=0)
    {
      CP_Pos[i] = 0.0;
    }
    else
    {
      double CP_g = g;
      double CP_xbar = CP_x + CP_xdot/sqrt(CP_g/CP_L);
      CP_Pos[i] = CP_xbar;
    }
  }
  double CPCECost = *min_element(CP_Pos.begin(), CP_Pos.end());
  CPCECost = max(0.0, -CPCECost);
  if(CPCECost>0)
  {
    // This indicates that robot's is going to have failure
    PIPIndex = std::distance(CP_Pos.begin(), std::min_element(CP_Pos.begin(), CP_Pos.end()));
  }
  else
  {
    PIPIndex = -1;
  }
  return CPCECost;
}

double ZMPGeneratorAnalysis(const std::vector<PIPInfo> & PIPTotal, const Vector3 & COMPos, const Vector3 & COMAcc, const double & Margin)
{
  std::vector<double> ZMP_Pos(PIPTotal.size());
  for (int i = 0; i < PIPTotal.size(); i++)
  {
    double L = PIPTotal[i].L;
    double theta = PIPTotal[i].theta;
    double thetadot = PIPTotal[i].thetadot;
    double g_angle = PIPTotal[i].g_angle;
    double CP_x = L * sin(theta);
    double CP_xdot = L * thetadot * cos(theta);
    double CP_L = L * cos(theta);
    double CP_g = 9.81;
    double ZMPxddot= dot(PIPTotal[i].z_prime_unit, COMAcc);
    double CP_xbar = CP_x - CP_L/CP_g * ZMPxddot - Margin;
    ZMP_Pos[i] = CP_xbar;
  }
  double CPCECost = *min_element(ZMP_Pos.begin(), ZMP_Pos.end());
  return max(0.0, -CPCECost);
}

static void Vector3Appender(std::vector<Vector3> & PIPInters, const Vector3 & Inter_i)
{
  // This function is used to append non-repeatative Vector3 to PIPInters
  double eps = 1e-8;
  int InterLen = PIPInters.size();
  switch (InterLen)
  {
    case 0:
    {
      PIPInters.push_back(Inter_i);
    }
    break;
    default:
    {
      std::vector<double> Inter_Value(InterLen);
      for (int i = 0; i < InterLen; i++)
      {
        Vector3 PIPInters_i = PIPInters[i];
        double Inter_Value_i_x = PIPInters_i[0] - Inter_i[0];
        double Inter_Value_i_y = PIPInters_i[1] - Inter_i[1];
        double Inter_Value_i_z = PIPInters_i[2] - Inter_i[2];

        double Inter_Value_i = Inter_Value_i_x * Inter_Value_i_x + Inter_Value_i_y * Inter_Value_i_y + Inter_Value_i_z * Inter_Value_i_z;
        Inter_Value[i] = Inter_Value_i;
      }

      double Inter_Value_min = *std::min_element(Inter_Value.begin(), Inter_Value.begin() + InterLen);
      if (Inter_Value_min>eps)
      {
        PIPInters.push_back(Inter_i);
      }
    }
    break;
  }
}

std::vector<Vector3> FullPIPInterCal(const std::vector<FacetInfo> & FacetInfoObj, const Vector3 & COM)
{
  // This function is used to calcualte the intersections of all the PIPs
  // Here a PIP projection funtion needs to be coded.
  std::vector<Vector3> PIPInters;
  for (int i = 0; i < FacetInfoObj.size(); i++)
  {
    for (int j = 0; j < FacetInfoObj[i].FacetEdges.size(); j++)
    {
      Vector3 COMVel(0.0, 0.0, 0.0);
      Vector3 EdgeA = FacetInfoObj[i].FacetEdges[j].first;
      Vector3 EdgeB = FacetInfoObj[i].FacetEdges[j].second;
      PIPInfo PIPMProj_i = PIPGeneratorInner(EdgeA, EdgeB, COM, COMVel);
      Vector3Appender(PIPInters, PIPMProj_i.Intersection);
    }
  }
  return PIPInters;
}

int Edge2EndEffector(Robot & SimRobot, const Vector3 & EdgePoint, const std::vector<LinkInfo> & RobotLinkInfo)
{
  // Enumerate all the contact point and gives out the one with the lowest deviation.
  std::vector<double> EdgeDev(RobotLinkInfo.size());
  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    std::vector<double> EdgeInnerDev(RobotLinkInfo[i].LocalContacts.size());
    for (int j = 0; j < RobotLinkInfo[i].LocalContacts.size(); j++)
    {
      // This means that current contact is active and we should keep its location and Jacobian.
      Vector3 LinkiPjPos;
      SimRobot.GetWorldPosition(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, LinkiPjPos);
      double LinkiPjPosDevEdge_x = LinkiPjPos.x - EdgePoint.x;
      double LinkiPjPosDevEdge_y = LinkiPjPos.y - EdgePoint.y;
      double LinkiPjPosDevEdge_z = LinkiPjPos.z - EdgePoint.z;
      LinkiPjPosDevEdge_z = 0.0;
      double LinkiPjPosDevEdge = LinkiPjPosDevEdge_x * LinkiPjPosDevEdge_x + LinkiPjPosDevEdge_y * LinkiPjPosDevEdge_y + LinkiPjPosDevEdge_z * LinkiPjPosDevEdge_z;
      EdgeInnerDev[j] = LinkiPjPosDevEdge;
    }
    // Get the minimum element out for EdgeDev
    EdgeDev[i] = *std::min_element(EdgeInnerDev.begin(), EdgeInnerDev.end());
  }
  int EdgeEffectorInfoIndex = std::distance(EdgeDev.begin(), std::min_element(EdgeDev.begin(), EdgeDev.end()));
  return EdgeEffectorInfoIndex;
}

double FailureMetricwithContactChange(Robot & SimRobot, const int & CriticalLink, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & _RobotContactInfo, Vector3 & Edge1, Vector3 & Edge2)
{
  std::vector<ContactStatusInfo> RobotContactInfo = _RobotContactInfo;
  for (int i = 0; i < RobotLinkInfo[CriticalLink].LocalContacts.size(); i++)
  {
    switch (_RobotContactInfo[CriticalLink].LocalContactStatus[i])
    {
      case 1:
      {
        RobotContactInfo[CriticalLink].LocalContactStatus[i] = 0;
      }
      break;
      default:
      {
        RobotContactInfo[CriticalLink].LocalContactStatus[i] = 1;
      }
      break;
    }
  }

  std::vector<Vector3>  ActContacts;
  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    for (int j = 0; j < RobotLinkInfo[i].LocalContacts.size(); j++)
    {
      switch (RobotContactInfo[i].LocalContactStatus[j])
      {
        case 1:
        {
          // This means that current contact is active and we should keep its location and Jacobian.
          Vector3 LinkiPjPos, LinkiPjVel;
          SimRobot.GetWorldPosition(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, LinkiPjPos);
          ActContacts.push_back(LinkiPjPos);
        }
        break;
        default:
        break;
      }
    }
  }

  /* Robot's COMPos and COMVel */
  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(SimRobot, COMPos, COMVel);
  std::vector<Vector3> ProjActContactPos = ProjActContactPosGene(ActContacts);
  std::vector<PIPInfo> PIPSPTotal = PIPGenerator(ProjActContactPos, COMPos, COMVel);
  int CPPIPIndex;
  double CPObjective = CapturePointGenerator(PIPSPTotal, CPPIPIndex);

  switch (CPPIPIndex)
  {
    case -1:
    {
      Edge1.x = 0.0;
      Edge1.y = 0.0;
      Edge1.z = 0.0;
      Edge2 = Edge1;
    }
    break;
    default:
    {
      Edge1 = PIPSPTotal[CPPIPIndex].EdgeA;
      Edge2 = PIPSPTotal[CPPIPIndex].EdgeB;
    }
    break;
  }
  return CPObjective;
}

double FailureMetricEval(Robot & SimRobot, std::vector<LinkInfo> & RobotLinkInfo, std::vector<ContactStatusInfo> & RobotContactInfo)
{
  // A direct evaluation of failure metric from robot state, RobotLinkInfo and RobotContactInfo.
  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(SimRobot, COMPos, COMVel);
  std::vector<Vector3> ActContactPos = ContactPositionFinder(SimRobot, RobotLinkInfo, RobotContactInfo);
  std::vector<PIPInfo> PIPTotal = PIPGenerator(ActContactPos, COMPos, COMVel);
  int CPPIPIndex;
  double CPObjective = CapturePointGenerator(PIPTotal, CPPIPIndex);
  return CPObjective;
}
