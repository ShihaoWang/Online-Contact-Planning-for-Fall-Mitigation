#ifndef ROBOTINFO_H
#define ROBOTINFO_H
#include <KrisLibrary/robotics/RobotDynamics3D.h>
#include <KrisLibrary/robotics/NewtonEuler.h>
#include <Interface/SimulationGUI.h>
#include <unsupported/Eigen/CXX11/Tensor>
#include <KrisLibrary/meshing/TriMeshTopology.h>
#include <KrisLibrary/math3d/geometry3d.h>
#include <KrisLibrary/geometry/CollisionMesh.h>
#include <KrisLibrary/geometry/PQP/src/PQP.h>
#include "Splines.h"

struct LinkInfo
{
  LinkInfo(){LinkIndex = -1;}
  LinkInfo(const int &link_index){ LinkIndex = link_index; }
  void AddLocalConact(const Vector3& local_contact){ LocalContacts.push_back(local_contact); }
  LinkInfo Update(const int &link_index){ return LinkInfo(link_index); }
  void AvgContactUpdate()
  {
    switch (LocalContacts.size())
    {
      case 0:
      {
        throw std::invalid_argument( "LocalContacts should have been initialized!" );
      }
      break;
      default:
      {
        Vector3 SumLocalContacts(0.0, 0.0, 0.0);
        for (int i = 0; i < LocalContacts.size(); i++)
        {
          SumLocalContacts.x = SumLocalContacts.x + LocalContacts[i].x;
          SumLocalContacts.y = SumLocalContacts.y + LocalContacts[i].y;
          SumLocalContacts.z = SumLocalContacts.z + LocalContacts[i].z;
        }
        AvgLocalContact.x = SumLocalContacts.x/LocalContacts.size();
        AvgLocalContact.y = SumLocalContacts.y/LocalContacts.size();
        AvgLocalContact.z = SumLocalContacts.z/LocalContacts.size();
      }
      break;
    }
  }
  int LinkIndex;
  std::vector<Vector3> LocalContacts;
  std::vector<Vector3> ContactPositions;
  std::vector<Vector3> ContactVelocities;
  std::vector<double> ContactDists;
  Vector3 AvgLocalContact;
};

struct ContactStatusInfo
{
  // This struct saves the information of the contact status of each link end effector
  ContactStatusInfo(){LinkIndex = -1;}
  ContactStatusInfo(const int &link_index){ LinkIndex = link_index; }
  void AddLocalConactStatus(const int & _contactstatus){ LocalContactStatus.push_back(_contactstatus); }
  ContactStatusInfo Update(const int &link_index){ return ContactStatusInfo(link_index); }
  void StatusSwitch(const int & Val)
  {
    for(int i = 0; i<LocalContactStatus.size(); i++)
    {
      LocalContactStatus[i] = Val;
    }
  }
  int LinkIndex;
  std::vector<int> LocalContactStatus;
};

struct FacetInfo
{
  // This struct is used to save the information for the FacetEdges and FacetNorm.
  FacetInfo(){};
  void FacetEdgesUpdate(const std::vector<std::pair<Vector3, Vector3>> &_FacetEdges)
  {
    FacetEdges = _FacetEdges;
  }
  void FacetNormUpdate(const Vector3& _FacetNorm)
  {
    FacetNorm = _FacetNorm;
  }
  double ProjPoint2EdgeDist(const Vector3& _Point)
  {
    std::vector<double> ProjPoint2Edge_vec(EdgeNorms.size());
    Vector3 Vertex2Point = _Point - FacetEdges[0].first;
    double Point2Facet = Vertex2Point.dot(FacetNorm);
    Vector3 Facet2Point = Point2Facet * FacetNorm;

    for (int i = 0; i < EdgeNorms.size(); i++)
    {
      Vertex2Point = _Point - FacetEdges[i].first;
      Vector3 Vertex2ProjPoint = Vertex2Point - Facet2Point;
      double ProjPoint2Edge_i = Vertex2ProjPoint.dot(EdgeNorms[i]);
      ProjPoint2Edge_vec[i] = ProjPoint2Edge_i;
    }
    return *min_element(ProjPoint2Edge_vec.begin(), ProjPoint2Edge_vec.end());
  }
  void EdgesUpdate()
  {
    // This function will update the Edges based on the FacetEdges
    Edges.reserve(EdgeNorms.size());
    EdgesDirection.reserve(EdgeNorms.size());
    for (int i = 0; i < EdgeNorms.size(); i++)
    {
      Vector3 Edge_i = FacetEdges[i].second - FacetEdges[i].first;
      Edges.push_back(Edge_i);
      Vector3 Edge_i_normalized;
      Edge_i.getNormalized(Edge_i_normalized);
      EdgesDirection.push_back(Edge_i_normalized);
    }
  }
  std::vector<std::pair<Vector3, Vector3>> FacetEdges;
  std::vector<Vector3> EdgeNorms;
  Vector3 FacetNorm;
  std::vector<Vector3> Edges;
  std::vector<Vector3> EdgesDirection;
  int VertexNumber;
};

struct TerrainInfo
{
  // Each terrain has a TerrainInfo struct
  TerrainInfo(){num_tris = -1;}
  int num_tris;
  std::vector<Tri> Tris;
  std::vector<Vector3> TriNormals;
  std::vector<int> Indices;
};

struct PIPInfo
{
  // This struct saves the information of the projected inverted pendulum from the CoM to the edge of convex polyhedron
  PIPInfo(){
    L = 0.25;         // The reference bound range is [0.25, 0.85]
    Ldot = 0.0;
    theta = 0.0;
    thetadot = 0.0;
    g = 9.81;
    g_angle = 0.0;
  }
  PIPInfo(double _L, double _Ldot, double _theta, double _thetadot, double _g, double _g_angle)
  {
    L = _L;
    Ldot = _Ldot;
    theta = _theta;
    thetadot = _thetadot;
    g = _g;
    g_angle = _g_angle;
  }

  double L, Ldot, theta, thetadot;
  double g, g_angle;
  Vector3 x_prime_unit, y_prime_unit, z_prime_unit;
  Vector3 x_unit, y_unit, z_unit;
  Vector3 EdgeA, EdgeB;                   // The Edge points from EdgeA to EdgeB.
  Vector3 Intersection;                    // The point where the COM intersects the edge.
};

struct RotationAxisInfo
{
  // This struct is used to save the information of the rotation axis
  RotationAxisInfo(){};
  RotationAxisInfo(const Robot& SimRobot, const PIPInfo &IPM, const vector<LinkInfo>& RobotLinkInfo, const int & _ContactPointNo)
  {
    EdgeA = IPM.EdgeA;
    EdgeB = IPM.EdgeB;

    ContactPointNo = _ContactPointNo;

    Vector3 g(0.0, 0.0, -9.81);

    Vector3 EdgeABPosD_ = cross(g, EdgeB - EdgeA);
    EdgeABPosD_.getNormalized(EdgeABPosD);

    std::vector<double> EdgeADiff(ContactPointNo);
    std::vector<double> EdgeBDiff(ContactPointNo);

    int CPIndex = 0;

    // The next job is it to figure out the which links these two points belong to.
    std::vector<std::pair<int, int>> IndexToLinkiPointj;
    IndexToLinkiPointj.reserve(ContactPointNo);
    for (int i = 0; i < RobotLinkInfo.size(); i++)
    {
      for (int j = 0; j < RobotLinkInfo[i].LocalContacts.size(); j++)
      {
        Vector3 Link_i_Point_j_Position;
        SimRobot.GetWorldPosition(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, Link_i_Point_j_Position);
        Vector3 Link_i_Point_j_PositiontoEdgeA, Link_i_Point_j_PositiontoEdgeB;
        Link_i_Point_j_PositiontoEdgeA = Link_i_Point_j_Position - EdgeA;
        Link_i_Point_j_PositiontoEdgeB = Link_i_Point_j_Position - EdgeB;
        EdgeADiff[CPIndex] = Link_i_Point_j_PositiontoEdgeA.norm();
        EdgeBDiff[CPIndex] = Link_i_Point_j_PositiontoEdgeB.norm();
        IndexToLinkiPointj.push_back(std::make_pair(i, j));
        CPIndex = CPIndex + 1;
      }
    }
    int EdgeAListPos = std::distance(EdgeADiff.begin(),min_element(EdgeADiff.begin(),EdgeADiff.end()));
    int EdgeBListPos = std::distance(EdgeBDiff.begin(),min_element(EdgeBDiff.begin(),EdgeBDiff.end()));

    LinkAIndex = IndexToLinkiPointj[EdgeAListPos].first;
    LocalPointAIndex = IndexToLinkiPointj[EdgeAListPos].second;

    LinkBIndex = IndexToLinkiPointj[EdgeBListPos].first;
    LocalPointBIndex = IndexToLinkiPointj[EdgeBListPos].second;

    // The robot's current COM position and velocity should also been recorded.
    Vector3 COM_Pos_ = SimRobot.GetCOM();
    Matrix PartialCOMParitialq;
    SimRobot.GetCOMJacobian(PartialCOMParitialq);
    double COMVel_x =0.0, COMVel_y =0.0, COMVel_z =0.0;
    // MatrixPrintResult(PartialCOMParitialq);
    for (int i = 0; i < PartialCOMParitialq.n; i++)
    {
      COMVel_x = COMVel_x + PartialCOMParitialq(0,i) * SimRobot.dq[i];
      COMVel_y = COMVel_y + PartialCOMParitialq(1,i) * SimRobot.dq[i];
      COMVel_z = COMVel_z + PartialCOMParitialq(2,i) * SimRobot.dq[i];
    }
    Vector3 COM_Vel_(COMVel_x, COMVel_y, COMVel_z);
    COM_Pos = COM_Pos_;
    COM_Vel = COM_Vel_;
  }
  Vector3 EdgeA, EdgeB;                           // The Edge points from EdgeA to EdgeB.
  double DistA, DistB;                            // The relative distance from end effector to the environment
  int LinkAIndex, LinkBIndex;                     // The indicies for the left link edge and right link edge.
  int LocalPointAIndex, LocalPointBIndex;         // The local contact points on the link
  int ContactPointNo;
  Vector3 COM_Pos, COM_Vel;
  Vector3 EdgeABPosD;
  std::vector<Vector3> StanceContact;             // The StanceContact should save the contact(s) not to be changed.
  std::vector<std::pair<int, int>> StanceLinkIndex;             // The StanceContact should save the contact(s) not to be changed.
};

struct ViabilityKernelInfo
{
  // This struct saves the information of the computed HJBDB information
  ViabilityKernelInfo()
  {
    LLow = 0.35;
    LUpp = 1.05;
    LdotLow = -3.0;
    LdotUpp = 3.0;
    ThetaLow = -3.1415926535897932384626/2;
    ThetaUpp = -1.0 * ThetaLow;
    ThetadotLow = -3.0;
    ThetadotUpp = -1.0 * ThetadotLow;
    L_Grids = 61;
    Ldot_Grids = 61;
    Theta_Grids = 61;
    Thetadot_Grids = 61;
    AngleLow = 0;
    AngleUpp = 80;
    AngleDiff = 10;
    AngleNum = 9;

    DeltaT = 0.1;
  }
  ViabilityKernelInfo(const std::vector<double> & VKPara)
  {
    // VKPara is a 16-1 double vector
    LLow = VKPara[0];
    LUpp = VKPara[1];
    LdotLow = VKPara[2];
    LdotUpp = VKPara[3];
    ThetaLow = VKPara[4];
    ThetaUpp = VKPara[5];
    ThetadotLow = VKPara[6];
    ThetadotUpp = VKPara[7];
    L_Grids = std::round(VKPara[8]);
    Ldot_Grids = std::round(VKPara[9]);
    Theta_Grids = std::round(VKPara[10]);
    Thetadot_Grids = std::round(VKPara[11]);
    AngleLow = std::round(VKPara[12]);
    AngleUpp = std::round(VKPara[13]);
    AngleDiff = std::round(VKPara[14]);
    DeltaT = VKPara[15];

    AngleNum = round((1.0 * AngleUpp - 1.0 * AngleLow)/(1.0 * AngleDiff)) + 1;
  }
  static double TriLinearInterpolation(const double& x_FloatIndex, const int & x_leftindex, const double& y_FloatIndex, const int & y_leftindex, const double& z_FloatIndex, const int & z_leftindex, const double& valA, const double& valB, const double& valC, const double& valD, const double& valE, const double& valF, const double& valG, const double& valH)
  {

    // Since this is a tri-linear interpolation, there are three ways to do the interpolation

    // Type1:
    // Along x-direction
    double valMAB = (x_FloatIndex - x_leftindex*1.0) * (valB - valA) + valA;
    double valMDC = (x_FloatIndex - x_leftindex*1.0) * (valC - valD) + valD;
    double valMEF = (x_FloatIndex - x_leftindex*1.0) * (valF - valE) + valE;
    double valMHG = (x_FloatIndex - x_leftindex*1.0) * (valG - valH) + valH;

    // Along y-drection
    double valMABDC = (y_FloatIndex - y_leftindex*1.0) * (valMDC - valMAB) + valMAB;
    double valMEFGH = (y_FloatIndex - y_leftindex*1.0) * (valMHG - valMEF) + valMEF;

    // Along z-direction
    double valMABCDEFHG = (z_FloatIndex - z_leftindex*1.0) * (valMEFGH - valMABDC) + valMABDC;
    //
    // // Type2:
    // // Along y-drection
    // double valMAD = (y_FloatIndex - y_leftindex*1.0) * (valD - valA) + valA;
    // double valMBC = (y_FloatIndex - y_leftindex*1.0) * (valC - valB) + valB;
    // double valMEH = (y_FloatIndex - y_leftindex*1.0) * (valH - valE) + valE;
    // double valMFG = (y_FloatIndex - y_leftindex*1.0) * (valG - valF) + valF;
    //
    // //Along x-direction
    // double valMADBC = (x_FloatIndex - x_leftindex * 1.0) * (valMBC - valMAD) + valMAD;
    // double valMEHFG = (x_FloatIndex - x_leftindex * 1.0) * (valMFG - valMEH) + valMEH;
    //
    // //Along z-direction
    // double valMADBCEHFG = (z_FloatIndex - z_leftindex*1.0) * (valMEHFG - valMADBC) + valMADBC;

    return valMABCDEFHG;
  }

  double ObjectiveValue(const Eigen::Tensor<float,4>& ObjDataset, const int& L_index, const int& Ldot_index, const int& Theta_index, const int& Thetadot_index, const double & Theta_k)
  {
    if(Theta_k>ThetaUpp)
    {
      return 0.0;
    }
    if(Theta_k<ThetaLow)
    {
      // This case is considered to be not safe.
      return -1.0;
    }
    double valObj = ObjDataset(L_index, Ldot_index, Theta_index, Thetadot_index);
    return valObj;
  }

  void StateFromIndex(const int & NextIndex, const int & L_index, const int & Ldot_index, const int & Theta_index, const int & Thetadot_index, double & Lplus, double & Ldotplus, double & Thetaplus, double & Thetadotplus)
  {
    switch (NextIndex)
    {
      case -2:
      {
        Lplus = L_vector[L_index];
        Ldotplus = Ldot_vector[Ldot_index];
        Thetaplus = Theta_vector[Theta_index];
        Thetadotplus = Thetadot_vector[Thetadot_index];
      };
      break;
      default:
      {
        // This function is used to give the next state.
        int a = Ldot_Grids * Theta_Grids * Thetadot_Grids;
        int b = Theta_Grids * Thetadot_Grids;
        int c = Thetadot_Grids;

        int i = floor(NextIndex/a);
        int j = floor((NextIndex - a * i)/b);
        int k = floor((NextIndex - a * i - b * j)/c);
        int l = NextIndex - a * i - b * j - c * k;

        Lplus = L_vector[i];
        Ldotplus = Ldot_vector[j];
        Thetaplus = Theta_vector[k];
        Thetadotplus = Thetadot_vector[l];
      }
      break;
    }
  }

  int NextState(const Eigen::Tensor<int,4> & NextIndexTensor, const int & L_Index, const int & Ldot_Index, const int & Theta_Index, const int & Thetadot_Index, double & Lplus, double & Ldotplus, double & Thetaplus, double & Thetadotplus)
  {
    int NextStateIndex = NextIndexTensor(L_Index, Ldot_Index, Theta_Index, Thetadot_Index);
    double LPlus_, LdotPlus_, ThetaPlus_, ThetadotPlus_;
    StateFromIndex(NextStateIndex, L_Index, Ldot_Index, Theta_Index, Thetadot_Index, LPlus_, LdotPlus_, ThetaPlus_, ThetadotPlus_);
    Lplus = LPlus_;
    Ldotplus = LdotPlus_;
    Thetaplus = ThetaPlus_;
    Thetadotplus = ThetadotPlus_;
    return NextStateIndex;
  }

  double Objectiveinterpolator(const PIPInfo & PIP, const Vector3 & COMPosCur)
  {
    // This function is used to interpolate the objective function value for HJB VKs.
    double L_k = PIP.L;
    double Ldot_k = PIP.Ldot;
    double Theta_k = PIP.theta;
    double Thetadot_k = PIP.thetadot;
    double Angle_k = PIP.g_angle;
    double Angle_FloatIndex = (Angle_k - AngleLow)/AngleDiff * 1.0;

    int Angle_Index;
    if (Angle_FloatIndex<1.0 * AngleLow)
    {
      Angle_Index = 0;
    }
    else
    {
      if(Angle_FloatIndex>(1.0 * AngleUpp - 1.0 * AngleDiff))
      {
        Angle_Index = AngleNum - 1;
      }
      else
      {
        Angle_Index = round((Angle_k - 1.0 * AngleLow)/(1.0 * AngleDiff));
      }
    }

    double L_FloatIndex =         (L_k - LLow)/L_unit * 1.0;
    double Ldot_FloatIndex =      (Ldot_k - LdotLow)/Ldot_unit * 1.0;
    double Theta_FloatIndex =     (Theta_k - ThetaLow)/Theta_unit * 1.0;
    double Thetadot_FloatIndex =  (Thetadot_k - ThetadotLow)/Thetadot_unit * 1.0;

    int L_leftindex =             std::floor(L_FloatIndex);
    int Ldot_leftindex =          std::floor(Ldot_FloatIndex);
    int Theta_leftindex =         std::floor(Theta_FloatIndex);
    int Thetadot_leftindex =      std::floor(Thetadot_FloatIndex);

    if(L_leftindex<0)
    {
      L_leftindex = 0;
    }
    else
    {
      if(L_leftindex>L_Grids-2)
      {
        L_leftindex = L_Grids-2;
      }
    }

    if(Ldot_leftindex<0)
    {
      Ldot_leftindex = 0;
    }
    else
    {
      if(Ldot_leftindex>Ldot_Grids-2)
      {
        Ldot_leftindex = Ldot_Grids-2;
      }
    }

    if(Theta_leftindex<0)
    {
      Theta_leftindex = 0;
    }
    else
    {
      if(Theta_leftindex>Theta_Grids-2)
      {
        Theta_leftindex = Theta_Grids-2;
      }
    }

    if(Thetadot_leftindex<0)
    {
      Thetadot_leftindex = 0;
    }
    else
    {
      if(Thetadot_leftindex>Thetadot_Grids-2)
      {
        Thetadot_leftindex = Thetadot_Grids-2;
      }
    }

    if((Theta_k>1.5708)||(Theta_k<-1.5708))
    {
      return 0.0;
    }

    // 4D interpolation
    int L_rightindex =        L_leftindex + 1;
    int Ldot_rightindex =     Ldot_leftindex + 1;
    int Theta_rightindex =    Theta_leftindex + 1;
    int Thetadot_rightindex = Thetadot_leftindex + 1;

    // Thetadot_leftindex
    double valA1 = ObjectiveValue(ObjVec[Angle_Index],    L_leftindex, Ldot_leftindex, Theta_leftindex, Thetadot_leftindex, Theta_k);
    double valB1 = ObjectiveValue(ObjVec[Angle_Index],    L_rightindex, Ldot_leftindex, Theta_leftindex, Thetadot_leftindex, Theta_k);
    double valC1 = ObjectiveValue(ObjVec[Angle_Index],    L_rightindex, Ldot_rightindex, Theta_leftindex, Thetadot_leftindex, Theta_k);
    double valD1 = ObjectiveValue(ObjVec[Angle_Index],    L_leftindex, Ldot_rightindex, Theta_leftindex, Thetadot_leftindex, Theta_k);

    double valE1 = ObjectiveValue(ObjVec[Angle_Index],    L_leftindex, Ldot_leftindex, Theta_rightindex, Thetadot_leftindex, Theta_k);
    double valF1 = ObjectiveValue(ObjVec[Angle_Index],    L_rightindex, Ldot_leftindex, Theta_rightindex, Thetadot_leftindex, Theta_k);
    double valG1 = ObjectiveValue(ObjVec[Angle_Index],    L_rightindex, Ldot_rightindex, Theta_rightindex, Thetadot_leftindex, Theta_k);
    double valH1 = ObjectiveValue(ObjVec[Angle_Index],    L_leftindex, Ldot_rightindex, Theta_rightindex, Thetadot_leftindex, Theta_k);
    double val1 = TriLinearInterpolation(L_FloatIndex,    L_leftindex, Ldot_FloatIndex, Ldot_leftindex, Theta_FloatIndex, Theta_leftindex, valA1, valB1, valC1, valD1, valE1, valF1, valG1, valH1);

    // Thetadot_rightindex
    double valA2 = ObjectiveValue(ObjVec[Angle_Index],    L_leftindex, Ldot_leftindex, Theta_leftindex, Thetadot_rightindex, Theta_k);
    double valB2 = ObjectiveValue(ObjVec[Angle_Index],    L_rightindex, Ldot_leftindex, Theta_leftindex, Thetadot_rightindex, Theta_k);
    double valC2 = ObjectiveValue(ObjVec[Angle_Index],    L_rightindex, Ldot_rightindex, Theta_leftindex, Thetadot_rightindex, Theta_k);
    double valD2 = ObjectiveValue(ObjVec[Angle_Index],    L_leftindex, Ldot_rightindex, Theta_leftindex, Thetadot_rightindex, Theta_k);

    double valE2 = ObjectiveValue(ObjVec[Angle_Index],    L_leftindex, Ldot_leftindex, Theta_rightindex, Thetadot_rightindex, Theta_k);
    double valF2 = ObjectiveValue(ObjVec[Angle_Index],    L_rightindex, Ldot_leftindex, Theta_rightindex, Thetadot_rightindex, Theta_k);
    double valG2 = ObjectiveValue(ObjVec[Angle_Index],    L_rightindex, Ldot_rightindex, Theta_rightindex, Thetadot_rightindex, Theta_k);
    double valH2 = ObjectiveValue(ObjVec[Angle_Index],    L_leftindex, Ldot_rightindex, Theta_rightindex, Thetadot_rightindex, Theta_k);
    double val2 = TriLinearInterpolation(L_FloatIndex,    L_leftindex, Ldot_FloatIndex, Ldot_leftindex, Theta_FloatIndex, Theta_leftindex, valA2, valB2, valC2, valD2, valE2, valF2, valG2, valH2);

    // // (1,0)
    // double valA3 = ObjectiveValue(ObjVec[Angle_Index],   L_leftindex, Ldot_leftindex, Theta_rightindex, Thetadot_leftindex);
    // double valB3 = ObjectiveValue(ObjVec[Angle_Index],   L_rightindex, Ldot_leftindex, Theta_rightindex, Thetadot_leftindex);
    // double valC3 = ObjectiveValue(ObjVec[Angle_Index],   L_rightindex, Ldot_rightindex, Theta_rightindex, Thetadot_leftindex);
    // double valD3 = ObjectiveValue(ObjVec[Angle_Index],   L_leftindex, Ldot_rightindex, Theta_rightindex, Thetadot_leftindex);
    //
    // double valE3 = ObjectiveValue(ObjVec[Angle_Index],   L_leftindex, Ldot_leftindex, Theta_rightindex, Thetadot_leftindex);
    // double valF3 = ObjectiveValue(ObjVec[Angle_Index],   L_rightindex, Ldot_leftindex, Theta_rightindex, Thetadot_leftindex);
    // double valG3 = ObjectiveValue(ObjVec[Angle_Index],   L_rightindex, Ldot_rightindex, Theta_rightindex, Thetadot_leftindex);
    // double valH3 = ObjectiveValue(ObjVec[Angle_Index],   L_leftindex, Ldot_rightindex, Theta_rightindex, Thetadot_leftindex);
    // double val3 = TriLinearInterpolation(L_FloatIndex, L_leftindex, Ldot_FloatIndex, Ldot_leftindex, Theta_FloatIndex, Theta_leftindex, valA3, valB3, valC3, valD3, valE3, valF3, valG3, valH3);
    //
    // // (1,1)
    // double valA4 = ObjectiveValue(ObjVec[Angle_Index],   L_leftindex, Ldot_leftindex, Theta_rightindex, Thetadot_rightindex);
    // double valB4 = ObjectiveValue(ObjVec[Angle_Index],   L_rightindex, Ldot_leftindex, Theta_rightindex, Thetadot_rightindex);
    // double valC4 = ObjectiveValue(ObjVec[Angle_Index],   L_rightindex, Ldot_rightindex, Theta_rightindex, Thetadot_rightindex);
    // double valD4 = ObjectiveValue(ObjVec[Angle_Index],   L_leftindex, Ldot_rightindex, Theta_rightindex, Thetadot_rightindex);
    //
    // double valE4 = ObjectiveValue(ObjVec[Angle_Index],   L_leftindex, Ldot_leftindex, Theta_rightindex, Thetadot_rightindex);
    // double valF4 = ObjectiveValue(ObjVec[Angle_Index],   L_rightindex, Ldot_leftindex, Theta_rightindex, Thetadot_rightindex);
    // double valG4 = ObjectiveValue(ObjVec[Angle_Index],   L_rightindex, Ldot_rightindex, Theta_rightindex, Thetadot_rightindex);
    // double valH4 = ObjectiveValue(ObjVec[Angle_Index],   L_leftindex, Ldot_rightindex, Theta_rightindex, Thetadot_rightindex);
    // double val4 = TriLinearInterpolation(L_FloatIndex, L_leftindex, Ldot_FloatIndex, Ldot_leftindex, Theta_FloatIndex, Theta_leftindex, valA4, valB4, valC4, valD4, valE4, valF4, valG4, valH4);

    // // Now we have a bi-linear interpolation
    // // Along theta direction
    // double val1M3 = (val3 - val1) * (Theta_FloatIndex - Theta_leftindex) + val1;
    // double val2M4 = (val4 - val2) * (Theta_FloatIndex - Theta_leftindex) + val2;
    // // Along thetadot direction
    // double val1M32M4 = (val2M4 - val1M3) * (Thetadot_FloatIndex - Thetadot_leftindex) + val1M3;

    double valFinal = (val2 - val1) * (Thetadot_FloatIndex - Thetadot_leftindex) + val1;

    return valFinal;
  }

  double ObjectiveRetrieve(const PIPInfo & PIP, const Vector3 & COMPosCur)
  {
    // This function is used to interpolate the objective value according to the gravitational projection
    double L_k = PIP.L;
    double Ldot_k = PIP.Ldot;
    double Theta_k = PIP.theta;
    double Thetadot_k = PIP.thetadot;
    double Angle_k = PIP.g_angle;
    double Angle_FloatIndex = (Angle_k - AngleLow)/AngleDiff * 1.0;

    if(L_k<LLow)
    {
      return -1.0;
    }
    if(Theta_k>ThetaUpp)
    {
      return 0.0;
    }

    int Angle_Index;

    if (Angle_FloatIndex<1.0 * AngleLow)
    {
      Angle_Index = 0;
    }
    else
    {
      if(Angle_FloatIndex>(1.0 * AngleUpp - 1.0 * AngleDiff))
      {
        Angle_Index = AngleNum - 1;
      }
      else
      {
        Angle_Index = round((Angle_k - 1.0 * AngleLow)/(1.0 * AngleDiff));
      }
    }

    // For the current project, there is no need to get a float value out since the main purpose is for stability validation.
    // Get the nearest neighbor
    double L_FloatIndex =         (L_k - LLow)/L_unit * 1.0;
    double Ldot_FloatIndex =      (Ldot_k - LdotLow)/Ldot_unit * 1.0;
    double Theta_FloatIndex =     (Theta_k - ThetaLow)/Theta_unit * 1.0;
    double Thetadot_FloatIndex =  (Thetadot_k - ThetadotLow)/Thetadot_unit * 1.0;

    int L_index =             std::round(L_FloatIndex);
    int Ldot_index =          std::round(Ldot_FloatIndex);
    int Theta_index =         std::round(Theta_FloatIndex);
    int Thetadot_index =      std::round(Thetadot_FloatIndex);

    if(L_index<0)
    {
      L_index = 0;
    }
    else
    {
      if(L_index>L_Grids-1)
      {
        L_index = L_Grids-1;
      }
    }

    if(Ldot_index<0)
    {
      Ldot_index = 0;
    }
    else
    {
      if(Ldot_index>Ldot_Grids-1)
      {
        Ldot_index = Ldot_Grids-1;
      }
    }

    if(Theta_index<0)
    {
      Theta_index = 0;
    }
    else
    {
      if(Theta_index>Theta_Grids-1)
      {
        Theta_index = Theta_Grids-1;
      }
    }

    if(Thetadot_index<0)
    {
      Thetadot_index = 0;
    }
    else
    {
      if(Thetadot_index>Thetadot_Grids-1)
      {
        Thetadot_index = Thetadot_Grids-1;
      }
    }
    double Obj = ObjectiveValue(ObjVec[Angle_Index],   L_index,  Ldot_index,   Theta_index,  Thetadot_index, Theta_k);
    // double Lplus, Ldotplus, Thetaplus, Thetadotplus;
    // int NextStateStatus = NextState(NextIndexVec[Angle_Index],L_index,Ldot_index,Theta_index,Thetadot_index,Lplus,Ldotplus,Thetaplus,Thetadotplus);
    // switch (NextStateStatus)
    // {
    //   case -2:
    //   {
    //     // In this case, there is no need for HJB to be utilized.
    //     COMPosDes = COMPosCur;
    //   }
    //   break;
    //   default:
    //   {
    //     Vector3 COMHJB = PIP.Intersection + Lplus * cos(Thetaplus) * PIP.y_prime_unit - Lplus * sin(Thetaplus) * PIP.z_prime_unit;
    //     Vector3 COMOff = COMHJB - COMPosCur;
    //
    //     // Option1: utilization of linear interpolation
    //     COMPosDes.x = COMPosCur.x + COMOff.x * dt/DeltaT;
    //     COMPosDes.y = COMPosCur.y + COMOff.y * dt/DeltaT;
    //     COMPosDes.z = COMPosCur.z + COMOff.z * dt/DeltaT;
    //
    //     // // Option2: utilization of next step position
    //     // COMPosDes = COMHJB;
    //   }
    //   break;
    // }
    return Obj;
  }
  double LLow, LUpp, LdotLow, LdotUpp, ThetaLow, ThetaUpp, ThetadotLow, ThetadotUpp;
  double L_unit, Ldot_unit, Theta_unit, Thetadot_unit;
  int L_Grids, Ldot_Grids, Theta_Grids, Thetadot_Grids;
  std::vector<double> L_vector, Ldot_vector, Theta_vector, Thetadot_vector;
  std::vector<Eigen::Tensor<float,4>> ObjVec;           // This is the vector of FailureMetric for each angle.
  std::vector<Eigen::Tensor<int,4>> NextIndexVec;                 // This vector saves the indices of the next index
  int AngleLow, AngleUpp, AngleDiff, AngleNum;                    // These three save the Angle Range for HJBDataBase.
  double DeltaT;                                                  // This is the time step for HJB computation.
};

struct SignedDistanceFieldInfo
{
  // This struct is used to save the information of the signed distance field
  SignedDistanceFieldInfo()
  {
    Envi_x_min = 0;           Envi_x_max = 0;
    Envi_y_min = 0;           Envi_y_max = 0;
    Envi_z_min = 0;           Envi_z_max = 0;
    Envi_x_unit = 0;          Envi_y_unit = 0;          Envi_z_unit = 0;
    Envi_x_length = 0;        Envi_y_length = 0;        Envi_z_length = 0;
    GridNo = 0;
  }
  SignedDistanceFieldInfo(const Eigen::Tensor<double, 3>& _SDFTensor, const std::vector<double> &_SDFSpecs)
  {
    SDFTensor = _SDFTensor;
    Envi_x_min = _SDFSpecs[0];          Envi_x_max = _SDFSpecs[1];
    Envi_y_min = _SDFSpecs[2];          Envi_y_max = _SDFSpecs[3];
    Envi_z_min = _SDFSpecs[4];          Envi_z_max = _SDFSpecs[5];
    Envi_x_unit = _SDFSpecs[6];         Envi_y_unit = _SDFSpecs[7];         Envi_z_unit = _SDFSpecs[8];
    Envi_x_length = _SDFSpecs[9];       Envi_y_length = _SDFSpecs[10];      Envi_z_length = _SDFSpecs[11];
    GridNo = (int)_SDFSpecs[12];
  }
  double SignedDistance(const Vector3 &Point) const
  {
    // This function is used to compute the distance from a 3D point to the environment terrain
    // The first job is to figure out the nearest neighbours of the Points

    double x_FloatIndex = (Point.x - Envi_x_min)/Envi_x_unit * 1.0;
    double y_FloatIndex = (Point.y - Envi_y_min)/Envi_y_unit * 1.0;
    double z_FloatIndex = (Point.z - Envi_z_min)/Envi_z_unit * 1.0;

    int x_leftindex = std::floor(x_FloatIndex);
    int y_leftindex = std::floor(y_FloatIndex);
    int z_leftindex = std::floor(z_FloatIndex);

    if(x_leftindex<0)
    {
      x_leftindex = 0;
    }
    else
    {
      if(x_leftindex>GridNo-2)
      {
        x_leftindex = GridNo-2;
      }
    }

    if(y_leftindex<0)
    {
      y_leftindex = 0;
    }
    else
    {
      if(y_leftindex>GridNo-2)
      {
        y_leftindex = GridNo-2;
      }
    }

    if(z_leftindex<0)
    {
      z_leftindex = 0;
    }
    else
    {
      if(z_leftindex>GridNo-2)
      {
        z_leftindex = GridNo-2;
      }
    }

    int x_rightindex = x_leftindex + 1;
    int y_rightindex = y_leftindex + 1;
    int z_rightindex = z_leftindex + 1;

    double valA = SDFTensor(x_leftindex, y_leftindex, z_leftindex);
    double valB = SDFTensor(x_rightindex, y_leftindex, z_leftindex);
    double valC = SDFTensor(x_rightindex, y_rightindex, z_leftindex);
    double valD = SDFTensor(x_leftindex, y_rightindex, z_leftindex);

    double valE = SDFTensor(x_leftindex, y_leftindex, z_rightindex);
    double valF = SDFTensor(x_rightindex, y_leftindex, z_rightindex);
    double valG = SDFTensor(x_rightindex, y_rightindex, z_rightindex);
    double valH = SDFTensor(x_leftindex, y_rightindex, z_rightindex);

    // Since this is a tri-linear interpolation, there are three ways to do the interpolation

    // Type1:
    // Along x-direction
    double valMAB = (x_FloatIndex - x_leftindex*1.0) * (valB - valA) + valA;
    double valMDC = (x_FloatIndex - x_leftindex*1.0) * (valC - valD) + valD;
    double valMEF = (x_FloatIndex - x_leftindex*1.0) * (valF - valE) + valE;
    double valMHG = (x_FloatIndex - x_leftindex*1.0) * (valG - valH) + valH;

    // Along y-drection
    double valMABDC = (y_FloatIndex - y_leftindex*1.0) * (valMDC - valMAB) + valMAB;
    double valMEFGH = (y_FloatIndex - y_leftindex*1.0) * (valMHG - valMEF) + valMEF;

    // Along z-direction
    double valMABCDEFHG = (z_FloatIndex - z_leftindex*1.0) * (valMEFGH - valMABDC) + valMABDC;

    // // Type2:
    // // Along y-drection
    // double valMAD = (y_FloatIndex - y_leftindex*1.0) * (valD - valA) + valA;
    // double valMBC = (y_FloatIndex - y_leftindex*1.0) * (valC - valB) + valB;
    // double valMEH = (y_FloatIndex - y_leftindex*1.0) * (valH - valE) + valE;
    // double valMFG = (y_FloatIndex - y_leftindex*1.0) * (valG - valF) + valF;
    //
    // //Along x-direction
    // double valMADBC = (x_FloatIndex - x_leftindex * 1.0) * (valMBC - valMAD) + valMAD;
    // double valMEHFG = (x_FloatIndex - x_leftindex * 1.0) * (valMFG - valMEH) + valMEH;
    //
    // //Along z-direction
    // double valMADBCEHFG = (z_FloatIndex - z_leftindex*1.0) * (valMEHFG - valMADBC) + valMADBC;

    return valMABCDEFHG;
  }
  Vector3 SignedDistanceNormal(const Vector3 &Point) const
  {
    // This function is used to calculate the (1 x 3) Jacobian matrix given the current Position
    // This function is used to compute the distance from a 3D point to the environment terrain
    // The first job is to figure out the nearest neighbours of the Points

    double x_FloatIndex = (Point.x - Envi_x_min)/Envi_x_unit * 1.0;
    double y_FloatIndex = (Point.y - Envi_y_min)/Envi_y_unit * 1.0;
    double z_FloatIndex = (Point.z - Envi_z_min)/Envi_z_unit * 1.0;

    int x_leftindex = std::floor(x_FloatIndex);
    int y_leftindex = std::floor(y_FloatIndex);
    int z_leftindex = std::floor(z_FloatIndex);

    if(x_leftindex<0)
    {
      x_leftindex = 0;
    }
    else
    {
      if(x_leftindex>GridNo-2)
      {
        x_leftindex = GridNo-2;
      }
    }

    if(y_leftindex<0)
    {
      y_leftindex = 0;
    }
    else
    {
      if(y_leftindex>GridNo-2)
      {
        y_leftindex = GridNo-2;
      }
    }

    if(z_leftindex<0)
    {
      z_leftindex = 0;
    }
    else
    {
      if(z_leftindex>GridNo-2)
      {
        z_leftindex = GridNo-2;
      }
    }

    int x_rightindex = x_leftindex + 1;
    int y_rightindex = y_leftindex + 1;
    int z_rightindex = z_leftindex + 1;

    double valA = SDFTensor(x_leftindex, y_leftindex, z_leftindex);
    double valB = SDFTensor(x_rightindex, y_leftindex, z_leftindex);
    double valC = SDFTensor(x_rightindex, y_rightindex, z_leftindex);
    double valD = SDFTensor(x_leftindex, y_rightindex, z_leftindex);

    double valE = SDFTensor(x_leftindex, y_leftindex, z_rightindex);
    double valF = SDFTensor(x_rightindex, y_leftindex, z_rightindex);
    double valG = SDFTensor(x_rightindex, y_rightindex, z_rightindex);
    double valH = SDFTensor(x_leftindex, y_rightindex, z_rightindex);

    // Since this is a tri-linear interpolation, there are three ways to do the interpolation

    /*
      Jacobian matrix in the x-direction
    */

    // Cut the point with a plane orthgonal to z axis
    double valMAE =  (z_FloatIndex - z_leftindex*1.0) * (valE - valA) + valA;
    double valMBF =  (z_FloatIndex - z_leftindex*1.0) * (valF - valB) + valB;
    double valMDH =  (z_FloatIndex - z_leftindex*1.0) * (valH - valD) + valD;
    double valMCG =  (z_FloatIndex - z_leftindex*1.0) * (valG - valC) + valC;
    // Cut the point with a plane orthgonal to y axis
    double valMAEDH = (y_FloatIndex - y_leftindex*1.0) * (valMDH - valMAE) + valMAE;
    double valMBFCG = (y_FloatIndex - y_leftindex*1.0) * (valMCG - valMBF) + valMBF;
    // The values at the edge give the jacobian to x
    double JacDistTo_x = (valMBFCG - valMAEDH)/Envi_x_unit;

    /*
      Jacobian matrix in the y-direction
    */
    double valMABEF = (x_FloatIndex - x_leftindex*1.0) * (valMBF - valMAE) + valMAE;
    double valMDHCG = (x_FloatIndex - x_leftindex*1.0) * (valMCG - valMDH) + valMDH;

    double JacDistTo_y = (valMBFCG - valMAEDH)/Envi_y_unit;

    /*
      Jacobian matrix in the z-direction
    */
    // Cut the point with a plane orthgonal to x axis
    double valMAB = (x_FloatIndex - x_leftindex*1.0) * (valB - valA) + valA;
    double valMDC = (x_FloatIndex - x_leftindex*1.0) * (valC - valD) + valD;
    double valMEF = (x_FloatIndex - x_leftindex*1.0) * (valF - valE) + valE;
    double valMHG = (x_FloatIndex - x_leftindex*1.0) * (valG - valH) + valH;
    // Cut the point with a plane orthgonal to y axis
    double valMABDC = (y_FloatIndex - y_leftindex*1.0) * (valMDC - valMAB) + valMAB;
    double valMEFHG = (y_FloatIndex - y_leftindex*1.0) * (valMHG - valMEF) + valMHG;
    // The values at the edge give the jacobian to z
    double JacDistTo_z = (valMEFHG - valMABDC)/Envi_z_unit;

    return Vector3(JacDistTo_x, JacDistTo_y, JacDistTo_z);
  }
  Eigen::Tensor<double, 3> SDFTensor;
  double Envi_x_min, Envi_x_max;
  double Envi_y_min, Envi_y_max;
  double Envi_z_min, Envi_z_max;
  double Envi_x_unit, Envi_y_unit, Envi_z_unit;
  double Envi_x_length, Envi_y_length, Envi_z_length;
  int GridNo;
};

struct RMPoint
{
  // This struct is used to save the information of a point in Reachability Map.
  RMPoint()
  {
  }
  RMPoint(const double & r, const Vector3 & Pos)
  {
    // Constructor
    Radius = r;
    Position = Pos;
    Direction = Position;
    double PositionLength = sqrt(Position.x * Position.x + Position.y * Position.y + Position.z * Position.z);
    Direction.x = Direction.x/PositionLength;
    Direction.y = Direction.y/PositionLength;
    Direction.z = Direction.z/PositionLength;
  }
  double Radius;
  Vector3 Position;
  Vector3 Direction;
};

struct ReachabilityMap
{
  // This struct is used to save the reachability map
  ReachabilityMap()
  {}
  ReachabilityMap(const std::map<int, std::vector<RMPoint>> & _RMLayers)
  {
    RMLayers = _RMLayers;
  };
  void ReachabilityMapPara(const double & _MaxRadius, const int & _LayerNumber, const int & _PointNumberOnInner, const double & _LayerDiff, const double & _MinRadius)
  {
    MaxRadius = _MaxRadius;
    LayerNumber = _LayerNumber;
    PointNumberOnInner = _PointNumberOnInner;
    LayerDiff = _LayerDiff;
    MinRadius = _MinRadius;
  }
  std::vector<Vector3> IdealReachablePointsFinder(Robot & SimRobot, const int & LinkInfoIndex)
  {
    std::vector<Vector3> ReachablePoints;
    ReachablePoints.reserve(TotalPoint);
    double PivotalLinkIndex = EndEffectorPivotalIndex[LinkInfoIndex];
    Vector3 RefPoint, ZeroPos(0.0, 0.0, 0.0);
    SimRobot.GetWorldPosition(ZeroPos, PivotalLinkIndex, RefPoint);
    for (int i = 0; i < LayerNumber; i++)
    {
      std::vector<RMPoint> RMLayer_i = RMLayers[i];
      for (int j = 0; j < RMLayer_i.size(); j++)
      {
        Vector3 RMPointPos = RMLayer_i[j].Position + RefPoint;
        ReachablePoints.push_back(RMPointPos);
      }
    }
    return ReachablePoints;
  }
  std::vector<Vector3> ReachablePointsFinder(Robot & SimRobot, const int & LinkInfoIndex, SignedDistanceFieldInfo & SDFInfo, const Vector3 & COMVel)
  {
    double Radius = EndEffectorRadius[LinkInfoIndex];
    double PivotalLinkIndex = EndEffectorPivotalIndex[LinkInfoIndex];

    Vector3 RefPoint, ZeroPos(0.0, 0.0, 0.0);
    SimRobot.GetWorldPosition(ZeroPos, PivotalLinkIndex, RefPoint);
    int ReachablePointNo = 0;
    return ReachablePointsGene(RefPoint, Radius, SDFInfo, ReachablePointNo, COMVel);
  }
  std::vector<Vector3> ReachablePointsFinder(Robot & SimRobot, const int & LinkInfoIndex, SignedDistanceFieldInfo & SDFInfo)
  {
    double Radius = EndEffectorRadius[LinkInfoIndex];
    double PivotalLinkIndex = EndEffectorPivotalIndex[LinkInfoIndex];

    Vector3 RefPoint, ZeroPos(0.0, 0.0, 0.0);
    SimRobot.GetWorldPosition(ZeroPos, PivotalLinkIndex, RefPoint);
    Vector3 COMVel(0.0, 0.0, 0.0);
    int ReachablePointNo = 0;
    return ReachablePointsGene(RefPoint, Radius, SDFInfo, ReachablePointNo, COMVel);
  }
  std::vector<Vector3> ReachablePointsGene(const Vector3 & RefPoint, const double & Radius, SignedDistanceFieldInfo & SDFInfo, int & ReachablePointNo, const Vector3 & COMVel)
  {
    // Here MinRadius is used for comparison while Radius is the maximum allowed radius of robot's end effector.
    // This function is used to get presumably active ReachablePointsGene() from all sampled points.
    const double DisTol = 0.005;        // 5mm as a signed distance tolerance.
    std::vector<Vector3> ReachablePoints;
    ReachablePoints.reserve(TotalPoint);
    ReachablePointNo = 0;
    int LayerIndex = 0;
    double LayerRadius = MinRadius;
    if(Radius>MaxRadius)
    {
      // Then LayerNumber can all be used.
      LayerIndex = LayerNumber-1;
    }
    else
    {
      while (LayerRadius<Radius)
      {
        LayerRadius+=LayerDiff;
        LayerIndex++;
      }
      LayerRadius-=LayerDiff;
    }

    for (int i = 0; i < LayerIndex; i++)
    {
      std::vector<RMPoint> RMLayer_i = RMLayers[i];
      for (int j = 0; j < RMLayer_i.size(); j++)
      {
        Vector3 RMPointPos = RMLayer_i[j].Position + RefPoint;
        double CurrentDist = SDFInfo.SignedDistance(RMPointPos);
        // if((CurrentDist>=0)&&(CurrentDist*CurrentDist<DisTol*DisTol))
        if(CurrentDist*CurrentDist<DisTol*DisTol)
        {
          Vector3 RMPointNormal = SDFInfo.SignedDistanceNormal(RMPointPos);
          double Proj = RMPointNormal.x * COMVel.x + RMPointNormal.y * COMVel.y + RMPointNormal.z * COMVel.z;
          if(Proj<=0.0)
          {
            ReachablePoints.push_back(RMPointPos);
            ReachablePointNo++;
          }
        }
      }
    }
    return ReachablePoints;
  }
  std::vector<Vector3> ContactFreePointsFinder(const double & radius, const std::vector<Vector3> & ReachablePoints,const std::vector<std::pair<Vector3, double>> & ContactFreeInfo)
  {
    // This function can only be called after ReachablePointsFinder() to reduce the extra point further.
    std::vector<Vector3> ContactFreePoints;
    ContactFreePoints.reserve(ReachablePoints.size());
    int ContactFreeNo = 0;
    for (int i = 0; i < ReachablePoints.size(); i++)
    {
      Vector3 ReachablePoint = ReachablePoints[i];
      bool ContactFreeFlag = true;
      int ContactFreeInfoIndex = 0;
      while (ContactFreeInfoIndex<ContactFreeInfo.size())
      {
        Vector3 RefPoint = ContactFreeInfo[ContactFreeInfoIndex].first;
        double Radius = ContactFreeInfo[ContactFreeInfoIndex].second;
        Vector3 PosDiff = ReachablePoint - RefPoint;
        double PosDiffDis = sqrt(PosDiff.x * PosDiff.x + PosDiff.y * PosDiff.y + PosDiff.z * PosDiff.z);
        if(PosDiffDis<=(Radius + radius))
        {
          ContactFreeFlag = false;
          break;
        }
        ContactFreeInfoIndex++;
      }
      switch (ContactFreeFlag)
      {
        case true:
        {
          ContactFreePoints.push_back(ReachablePoints[i]);
          ContactFreeNo++;
        }
        break;
        default:
        {

        }
        break;
      }
    }
    return ContactFreePoints;
  };
  std::map<int, std::vector<RMPoint>> RMLayers;       // Each layer contains several data points.
  std::vector<double> EndEffectorCollisionRadius;
  std::vector<double> EndEffectorRadius;
  std::vector<int> EndEffectorLinkIndex;              //
  std::vector<int> EndEffectorPivotalIndex;
  std::map<int, std::vector<int>> EndEffectorLink2Pivotal;       // This map saves intermediate joint from End Effector Joint to Pivotal Joint.
  double MaxRadius;
  int LayerNumber;
  int PointNumberOnInner;
  double LayerDiff;
  double MinRadius;
  int TotalPoint;
};

struct SplineInfo
{
  SplineInfo(){};

  SplineInfo(const double & _sStart, const double & _sEnd, const Vector3 &_a, const Vector3 &_b, const Vector3 &_c, const Vector3 &_d)
  {
    sStart = _sStart;     sEnd = _sEnd;
    a = _a;    b = _b;    c = _c;    d = _d;
  };

  Vector3 SplinePosVector(const double & s)
  {
    Vector3 Pos = a * s * s * s + b * s * s + c * s + d;
    return Pos;
  }

  Vector3 SplineVelVector(const double & s)
  {
    Vector3 Vel = 3 * a * s *s + 2 * b * s + c;
    return Vel;
  }

  double sStart, sEnd;
  Vector3 a, b, c, d;

};

struct EndPathInfo
{
  // This info saves robot's Spline Object.
  EndPathInfo(){};
  EndPathInfo(const std::vector<SplineLib::cSpline3> & _SplineObj, const int & _EndEffectorIndex)
  {
    SplineObj = _SplineObj;
    EndEffectorIndex = _EndEffectorIndex;

    SplineNumber = SplineObj.size();

    SplineIndLength.reserve(SplineNumber);
    SplineSumLength.reserve(SplineNumber);

    TotalLength = 0.0f;
    for (int i = 0; i < SplineNumber; i++)
    {
        float len = Length(SplineObj[i], 0.01f);
        SplineIndLength.push_back(len);
        TotalLength += len;
        SplineSumLength.push_back(TotalLength);
    }

    SplineLib::Vec3f GoalPos = Position(SplineObj[SplineNumber-1], 1.0);
    GoalContactPos.x = GoalPos.x;
    GoalContactPos.y = GoalPos.y;
    GoalContactPos.z = GoalPos.z;
  }
  void s2Pos(const double & s, Vector3 & Pos)
  {
    // Here this function is used to get the corresponding position given the total s.
    double sLength = s * TotalLength;
    // Enumerate SplineSumLength to get the segment index
    if(sLength>TotalLength)
    {
      sLength = TotalLength;
    }
    if(s<0)
    {
      sLength = 0.0;
    }

    int SegmentNo = 0;
    while (SegmentNo<SplineNumber-1)
    {
      if(SplineSumLength[SegmentNo]>=sLength)
      {
        break;
      }
      SegmentNo++;
    }

    double ResLength = 0.0;
    switch (SegmentNo)
    {
      case 0:
      {
        ResLength = sLength;
      }
      break;
      default:
      {
        ResLength = sLength - SplineSumLength[SegmentNo-1];
      }
      break;
    }

    float sLength_i = ResLength/SplineIndLength[SegmentNo];
    SplineLib::Vec3f ps = Position(SplineObj[SegmentNo], sLength_i);
    Pos.x = ps.x;
    Pos.y = ps.y;
    Pos.z = ps.z;
  }
  void PosNTang(const double & s, Vector3 & Pos, Vector3 & Tang)
  {
    // Here this function is used to get the corresponding position given the total s.
    double sLength = s * TotalLength;
    // Enumerate SplineSumLength to get the segment index
    if(sLength>TotalLength)
    {
      sLength = TotalLength;
    }
    if(s<0)
    {
      sLength = 0.0;
    }

    int SegmentNo = 0;
    while (SegmentNo<SplineNumber-1)
    {
      if(SplineSumLength[SegmentNo]>=sLength)
      {
        break;
      }
      SegmentNo++;
    }

    double ResLength = 0.0;
    switch (SegmentNo)
    {
      case 0:
      {
        ResLength = sLength;
      }
      break;
      default:
      {
        ResLength = sLength - SplineSumLength[SegmentNo-1];
      }
      break;
    }

    float sLength_i = ResLength/SplineIndLength[SegmentNo];

    SplineLib::Vec3f ps = Position(SplineObj[SegmentNo], sLength_i);
    SplineLib::Vec3f vs = Velocity(SplineObj[SegmentNo], sLength_i);

    // This part is used for debugging.
    int SplineIndex;
    SplineLib::cSpline3 splines[SplineNumber];
    for (int i = 0; i < SplineNumber; i++)
    {
      splines[i] = SplineObj[i];
    }
    float s_i = FindClosestPoint(ps, SplineNumber, splines, &SplineIndex);

    Pos.x = ps.x;
    Pos.y = ps.y;
    Pos.z = ps.z;

    Tang.x = vs.x;
    Tang.y = vs.y;
    Tang.z = vs.z;
    double TangMag = sqrt(Tang.x * Tang.x + Tang.y * Tang.y + Tang.z * Tang.z);
    Tang.x = Tang.x/TangMag;
    Tang.y = Tang.y/TangMag;
    Tang.z = Tang.z/TangMag;
  }

  double Pos2s(const Vector3 & Pos)
  {
    // This function is used to compute the corresponding sTotal from Euclidean position.
    SplineLib::cSpline3 splines[SplineNumber];
    for (int i = 0; i < SplineNumber; i++)
    {
      splines[i] = SplineObj[i];
    }
    SplineLib::Vec3f qp(Pos.x, Pos.y, Pos.z);
    int SplineIndex;
    float s_i = FindClosestPoint(qp, SplineNumber, splines, &SplineIndex);
    double s = 0.0;
    switch (SplineIndex)
    {
      case 0:
      {
        double sLength = s_i * SplineIndLength[0];
        s = sLength/TotalLength;
      }
      break;
      default:
      {
        double sLength = s_i * SplineIndLength[SplineIndex] + SplineSumLength[SplineIndex-1];
        s = sLength/TotalLength;
      }
      break;
    }
    return s;
  }
  std::vector<SplineLib::cSpline3> SplineObj;
  std::vector<double> SplineIndLength;    // This saves the individual length for each piecewise curve.
  std::vector<double> SplineSumLength;    // This saves the accumulated length from the starting point along the curve.
  double TotalLength;
  int EndEffectorIndex;
  int SplineNumber;
  Vector3 GoalContactPos;                 // This saves the goal contact position.
};

struct AllContactStatusInfo
{
  // This struct saves the information of the contact status of each link end effector
  AllContactStatusInfo(){};
  void ContactStatusAppender(std::vector<ContactStatusInfo> & _ContactStatusInfoVec, const int & SwingLimbIndex)
  {
    // Here the SwingLimbIndex is the sequence index not the link index.
    ContactStatusInfoVec.push_back(_ContactStatusInfoVec);
    SwingLimbIndices.push_back(SwingLimbIndex);
  };
  std::vector<std::vector<ContactStatusInfo>> ContactStatusInfoVec;
  std::vector<int> SwingLimbIndices;
};


struct ControlReferenceInfo
{
  ControlReferenceInfo(){ControlReferenceFlag = false;};
  void TrajectoryUpdate(const std::vector<Config> & _ConfigTraj, const std::vector<double> & _TimeTraj, const std::vector<Vector3> & _SwingLimbTraj)
  {
    // The three trajectories have been provided.
    ConfigTraj = _ConfigTraj;
    TimeTraj = _TimeTraj;
    SwingLimbTraj = _SwingLimbTraj;
    ControlReferenceFlag = true;
    Impulse = 0.0;
    PlanningTime = 0.0;
  }

  void TimeBound(const double & Time, int & InitIndex, int & EndIndex)
  {
    // This function finds the index for Time within TimeTraj.
    for(int i = 0; i<TimeTraj.size(); i++)
    {
      if(TimeTraj[i]>Time)
      {
        InitIndex = i - 1;
        EndIndex = i;
        break;
      }
    }
  }

  std::vector<double> ConfigReference(const double & InitTime, const double & CurTime)
  {
    // This function is used to generate robot's current reference based on its trajetories.
    double TimeDur = CurTime - InitTime;
    if(CurTime<InitTime)
    {
      return ConfigTraj[0];
    }
    else
    {
      if((CurTime - InitTime)>TimeTraj[TimeTraj.size()-1])
      {
        return ConfigTraj[ConfigTraj.size()-1];
      }
      else
      {
        int InitIndex, EndIndex;
        TimeBound(TimeDur, InitIndex, EndIndex);
        // Then a linear interpolation will be conducted to get robot's current configuration reference.
        double TimeDurRef = TimeTraj[EndIndex] - TimeTraj[InitIndex];

        std::vector<double> ConfigVec(ConfigTraj[0].size());
        for(int i = 0; i < ConfigTraj[0].size(); i++)
        {
          double slope = (ConfigTraj[EndIndex][i] - ConfigTraj[InitIndex][i])/TimeDurRef;
          double ConfigVec_i = slope * (TimeDur - TimeTraj[InitIndex]) + ConfigTraj[InitIndex][i];
          ConfigVec[i] = ConfigVec_i;
        }
        return ConfigVec;
      }
    }
  }

  int SwingLimbIndex;
  std::vector<Config> ConfigTraj;         // This saves robot's reference trajectory.
  std::vector<double> TimeTraj;           // This saves the optimal time trajectory.
  std::vector<Vector3> SwingLimbTraj;     // This save the trajectory for robot's end effector
  bool ControlReferenceFlag;
  double Impulse;
  std::vector<ContactStatusInfo> InitContactInfo;
  std::vector<ContactStatusInfo> GoalContactInfo;
  double PlanningTime;
};

struct InvertedPendulumInfo
{
  InvertedPendulumInfo(){};
  InvertedPendulumInfo(const double & _Theta, const double & _Thetadot, const Vector3 & _COMPos, const Vector3 & _COMVel)
  {
    Theta = _Theta;
    Thetadot = _Thetadot;
    COMPos = _COMPos;
    COMVel = _COMVel;
  };

  double Theta;
  double Thetadot;
  Vector3 COMPos;
  Vector3 COMVel;
};

struct FailureStateInfo
{
  FailureStateInfo()
  {
    FailureTime = 0.0;
    FailureInitFlag = false;
  };
  void FailureStateUpdate(const double & _FailureTime, const Config & _FailureConfig, const Config & _FailureVelocity)
  {
    FailureTime = _FailureTime;
    FailureConfig = _FailureConfig;
    FailureVelocity = _FailureVelocity;
    FailureInitFlag = true;
  };
  double FailureTime;
  Config FailureConfig;
  Config FailureVelocity;
  bool FailureInitFlag;
};
#endif
