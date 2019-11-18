#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"

static std::vector<std::pair<Vector3, double>> ContactFreeInfoFn(const Robot & SimRobot, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject)
{
  // This function is used to generate pairs of ContactFreeInfo
  std::vector<std::pair<Vector3, double>> ContactFreeInfo;
  for (int i = 0; i < RobotContactInfo.size(); i++)
  {
    switch (RobotContactInfo[i].LocalContactStatus[0])
    {
      case 1:
      {
        Vector3 LinkiPjPos;
        SimRobot.GetWorldPosition(RobotLinkInfo[i].AvgLocalContact, RobotLinkInfo[i].LinkIndex, LinkiPjPos);
        double Radius = RMObject.EndEffectorCollisionRadius[i];
        auto ContactFreeInfo_i = std::make_pair (LinkiPjPos, Radius);
        ContactFreeInfo.push_back(ContactFreeInfo_i);
      }
      break;
    }
  }
  return ContactFreeInfo;
}

static std::vector<double> SingleContactPlanning(const int & LinkInfoIndex, const int & Type, const Robot & _SimRobot, const double & RefFailureMetric, const Vector3 & COMVel, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & _RobotContactInfo, ReachabilityMap & RMObject)
{
  /*
    This function is used to plan single contactL: _RobotContactInfo keeps the robot presumably contact status.
    Here Type will have two values:
    0     This indicates a contact modification.
    1     This indicates a contact addition.
  */

  Robot SimRobot = _SimRobot;
  std::vector<ContactStatusInfo> OriRobotContactInfo = _RobotContactInfo;
  switch (Type)
  {
    case 0:
    {
      // This is a contact modification.
      for (int i = 0; i < RobotLinkInfo[LinkInfoIndex].LocalContacts.size(); i++)
      {
        OriRobotContactInfo[LinkInfoIndex].LocalContactStatus[i] = 0;
      }
    }
    break;
    default:
    {}
    break;
  }
  // Now the job is to get all the active contact for planning purpose.
  std::vector<Vector3> FixedContacts;
  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    for (int j = 0; j < RobotLinkInfo[i].LocalContacts.size(); j++)
    {
      switch (OriRobotContactInfo[i].LocalContactStatus[j])
      {
        case 1:
        {
          // This means that current contact is active and we should keep its location and Jacobian.
          Vector3 LinkiPjPos;
          SimRobot.GetWorldPosition(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, LinkiPjPos);
          FixedContacts.push_back(LinkiPjPos);
        }
        break;
        default:
        break;
      }
    }
  }

  std::vector<std::pair<Vector3, double>> ContactFreeInfo = ContactFreeInfoFn(SimRobot, RobotLinkInfo, OriRobotContactInfo, RMObject);


  // The following three are used to plot the all, active, and optimal
  int ReachablePointNumber1, ReachablePointNumber2;
  std::vector<Vector3> IdealReachableContact = RMObject.IdealReachablePointsFinder(SimRobot, LinkInfoIndex);
  Vector3Writer(IdealReachableContact, "IdealReachableContact");

  std::vector<Vector3> ActiveReachableContact = RMObject.ReachablePointsFinder(SimRobot, LinkInfoIndex, NonlinearOptimizerInfo::SDFInfo, ReachablePointNumber1);
  Vector3Writer(ActiveReachableContact, "ActiveReachableContact");

  int ContactFreeNo;
  std::vector<Vector3> ContactFreeContact =  RMObject.ContactFreePointsFinder(ActiveReachableContact, ContactFreeInfo, ContactFreeNo);


  for (int i = 0; i < ActiveReachableContact.size(); i++)
  {
    std::vector<double> RobotConfig;
    int FeasiRes = ContactFeasibleOptFn(SimRobot, LinkInfoIndex, ActiveReachableContact[i], RobotLinkInfo, RMObject, RobotConfig);
  }

  // std::vector<Vector3> ReachablePoints2 = RMObject.ReachablePointsFinder(SimRobot, LinkInfoIndex, SDFInfo, ReachablePointNumber2, COMVel);
  int a = 1;
  // There are four rules to get rid of unnecessary points.
}


static std::vector<Config> StabilizingPoseGenerator(const std::vector<int> & ContactModiInfoIndices, const std::vector<int> & ContactAddInfoIndices, const Robot & SimRobot, const double & RefFailureMetric, const Vector3 & COMVel, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject)
{
  std::vector<Config> StabilizingPoses;
  switch (ContactModiInfoIndices.size())
  {
    case 0:
    {
    }
    break;
    default:
    {
      for (int i = 0; i < ContactModiInfoIndices.size(); i++)
      {
        std::vector<double> StabilizingPose = SingleContactPlanning(ContactModiInfoIndices[i], 0, SimRobot, RefFailureMetric, COMVel, RobotLinkInfo, RobotContactInfo, RMObject);
        StabilizingPoses.push_back(Config(StabilizingPose));
      }
    }
    break;
  }
  switch (ContactAddInfoIndices.size())
  {
    case 0:
    {
    }
    break;
    default:
    {
      for (int i = 0; i < ContactAddInfoIndices.size(); i++)
      {
        std::vector<double> StabilizingPose = SingleContactPlanning(ContactAddInfoIndices[i], 1, SimRobot, RefFailureMetric, COMVel, RobotLinkInfo, RobotContactInfo, RMObject);
        StabilizingPoses.push_back(Config(StabilizingPose));
      }
    }
    break;
  }
  return StabilizingPoses;
}

int EndEffectorFixer(Robot & SimRobot, const PIPInfo & PIPObj, const double & RefFailureMetric, const Vector3 & COMVel, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject)
{
  // This function is used to fix the end effector whose contact should not be modified.
  // First the job is to figure out whether this edge belongs to a certain end effector

  double ContactModiColTime = -1.0;
  double ContactAddColTime = -1.0;

  // Evaluation of the current contact status for contact addition consideration.
  std::vector<int> ContactAddInfoIndices;
  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    switch (RobotContactInfo[i].LocalContactStatus[0])
    {
      case 0:
      {
        // This indicates that there are potentially available end effector to make contact.
        ContactAddInfoIndices.push_back(i);
      }
      break;
      default:
      {}
      break;
    }
  }

  // This part is to figure out what contact can be changed.
  std::vector<Vector3> ModiCOMPosTraj, ModiCOMVelTraj;
  int FixedContactIndex;;
  ContactModiColTime = ContactModiPreEstimation(SimRobot, PIPObj, RobotLinkInfo, RobotContactInfo, NonlinearOptimizerInfo::SDFInfo, FixedContactIndex, ModiCOMPosTraj, ModiCOMVelTraj);
  std::vector<int> ContactModiInfoIndices;
  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    switch (RobotContactInfo[i].LocalContactStatus[0])
    {
      case 1:
      {
        // This indicates that there are potentially available end effector(s) to make contact.
        if(i == FixedContactIndex)
        {

        }
        else
        {
          ContactModiInfoIndices.push_back(i);
        }
      }
      break;
      default:
      break;
    }
  }

  switch (ContactAddInfoIndices.size())
  {
    case 0:
    {
      // This means that all current contacts are active so no additional contact can be added!
    }
    break;
    default:
    {
      std::vector<Vector3> AddCOMPosTraj, AddCOMVelTraj;
      ContactAddColTime = ContactAddPreEstimation(SimRobot, PIPObj, NonlinearOptimizerInfo::SDFInfo, AddCOMPosTraj, AddCOMVelTraj);
    }
    break;
  }

  std::vector<Config> StabilizingPoses = StabilizingPoseGenerator(ContactModiInfoIndices, ContactAddInfoIndices, SimRobot, RefFailureMetric, COMVel, RobotLinkInfo, RobotContactInfo, RMObject);

  int a = 1;
  return 1;
}
