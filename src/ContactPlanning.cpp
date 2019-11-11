#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"

static double FloatTol = 1e-7;

static int Edge2EndEffector(Robot & SimRobot, const Vector3 & EdgePoint, const std::vector<LinkInfo> & RobotLinkInfo)
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

int EndEffectorFixer(Robot & SimRobot, const PIPInfo & PIPObj, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & RobotContactInfo, SignedDistanceFieldInfo & SDFInfo)
{
  // This function is used to fix the end effector whose contact should not be modified.
  // First the job is to figure out whether this edge belongs to a certain end effector
  Vector3 EdgeA = PIPObj.EdgeA;
  Vector3 EdgeB = PIPObj.EdgeB;

  int EdgeAEndInfoIndex = Edge2EndEffector(SimRobot, EdgeA, RobotLinkInfo);
  int EdgeBEndInfoIndex = Edge2EndEffector(SimRobot, EdgeB, RobotLinkInfo);

  int FixerInfoIndex = -1;
  Vector3 RotEdgeOri, RotEdgeGoal;
  if(EdgeAEndInfoIndex == EdgeBEndInfoIndex)
  {
    // This means that EdgeA and EdgeB are on the same end effector so the other three end effectors can be used for contact planning.
    FixerInfoIndex = EdgeAEndInfoIndex;
    RotEdgeOri = EdgeA;
    RotEdgeGoal = EdgeB;
  }
  else
  {
    // This indicates the current end is owned by two end effectors so we should be more careful about the selection of which end effector to be modified.
    // Basically the main idea is to alter the link where the rest of the link has a lower failure metric.
    Vector3 RotKeepBOri, RotKeepBGoal;
    double KeepBFixedCost = FailureMetricwithContactChange(SimRobot, EdgeAEndInfoIndex, RobotLinkInfo, RobotContactInfo, RotKeepBOri, RotKeepBGoal);
    Vector3 RotKeepAOri, RotKeepAGoal;
    double KeepAFixedCost = FailureMetricwithContactChange(SimRobot, EdgeBEndInfoIndex, RobotLinkInfo, RobotContactInfo, RotKeepAOri, RotKeepAGoal);

    if(KeepBFixedCost>KeepAFixedCost)
    {
      // Here A should be kept fixed.
      FixerInfoIndex = EdgeAEndInfoIndex;
      RotEdgeOri = RotKeepAOri;
      RotEdgeGoal = RotKeepAGoal;
    }
    else
    {
      // Here B should be kept fixed.
      FixerInfoIndex = EdgeBEndInfoIndex;
      RotEdgeOri = RotKeepBOri;
      RotEdgeGoal = RotKeepBGoal;
    }
  }
  int a = 1;
  /* Robot's COMPos and COMVel */
  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(SimRobot, COMPos, COMVel);
  double atime = CollisionTimeEstimator(RotEdgeOri, RotEdgeGoal, COMPos, COMVel, SDFInfo);

  // Now the job is to plan the contact with the available contact


}
