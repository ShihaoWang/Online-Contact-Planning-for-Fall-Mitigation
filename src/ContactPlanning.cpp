#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <ctime>
#include <algorithm>
#include <random>

static double Tol = 1e-5;

static double NewContactsEval(const std::vector<Vector3> & FixedContacts, const std::vector<Vector3> & NewContacts, const Vector3 & COMPos, const Vector3 & COMVel)
{
  // This function is used to evaluate the robot current NewContact according to our objective.
  std::vector<Vector3> ActContacts;
  ActContacts.reserve(FixedContacts.size() + NewContacts.size());
  for (int i = 0; i < FixedContacts.size(); i++)
  {
    ActContacts.push_back(FixedContacts[i]);
  }
  for (int i = 0; i < NewContacts.size(); i++)
  {
    ActContacts.push_back(NewContacts[i]);
  }
  std::vector<PIPInfo> PIPTotal = PIPGenerator(ActContacts, COMPos, COMVel);
  int CPPIPIndex;
  double CPObjective = CapturePointGenerator(PIPTotal, CPPIPIndex);
  return CPObjective;
}

static void FailureMetricEstimator(const std::vector<Vector3> & SupportContact, std::vector<Vector3> & SafeContact, std::vector<Vector3> & BetterContact, const std::vector<Vector3> & FixedContacts, const Vector3 & COMPos, const Vector3 & COMVel, const Vector3 & RefContact, const double & RefFailureMetric)
{
  // This function is used to estimate robot's cost based on the point contact addition for failure metric evaluation.
  SafeContact.reserve(SupportContact.size());
  BetterContact.reserve(SupportContact.size());

  const int ActContactNo = FixedContacts.size() + 1;
  for (int i = 0; i < SupportContact.size(); i++)
  {
    std::vector<Vector3> ActContacts;
    ActContacts.reserve(ActContactNo);
    for (int j = 0; j < ActContactNo-1; j++)
    {
      ActContacts.push_back(FixedContacts[j]);
    }
    ActContacts.push_back(SupportContact[i]);
    std::vector<PIPInfo> PIPTotal = PIPGenerator(ActContacts, COMPos, COMVel);
    int CPPIPIndex;
    double CPObjective = CapturePointGenerator(PIPTotal, CPPIPIndex);
    if(CPObjective<RefFailureMetric)
    {
      // This indicates that the curent contact is at least a better contact.
      if(CPObjective<Tol)
      {
        // This means that current contact is SafeContact
        SafeContact.push_back(SupportContact[i]);
      }
      else
      {
        // This means that current contact is BetterContact
        BetterContact.push_back(SupportContact[i]);
      }
    }
  }

  return;
}

// static void OptimalContactFinder(const Robot& SimRobot, const int & LinkInfoIndex, const std::vector<Vector3> & SupportContact, std::vector<ContactConfigInfo> & SafeContact, std::vector<ContactConfigInfo> & BetterContact, const std::vector<Vector3> & FixedContacts, const Vector3 & COMPos, const Vector3 & COMVel, const std::vector<LinkInfo> & RobotLinkInfo, ReachabilityMap & RMObject)
// {
//   for (int i = 0; i < SupportContact.size(); i++)
//   {
//     std::vector<double> RobotConfig;
//     std::vector<Vector3> NewContacts;
//     int FeasiRes = ContactFeasibleOptFn(SimRobot, LinkInfoIndex, SupportContact[i], RobotLinkInfo, RMObject, RobotConfig, NewContacts);
//     switch (FeasiRes)
//     {
//       case 1:
//       {
//         // This indicates that current Support Contact is reachable.
//         double NewContactFailureMetric = NewContactsEval(FixedContacts, NewContacts, COMPos, COMVel);
//         if (NewContactFailureMetric<RefFailureMetric)
//         {
//           if(NewContactFailureMetric<Tol)
//           {
//             ContactConfigInfo NewContactConfig(NewContacts, RobotConfig, SupportContact[i]);
//             SafeContact.push_back(NewContactConfig);
//           }
//           else
//           {
//             ContactConfigInfo NewContactConfig(NewContacts, RobotConfig, SupportContact[i]);
//             BetterContact.push_back(NewContactConfig);
//           }
//         }
//       }
//       break;
//       default:
//       {
//       }
//       break;
//     }
//   }
//   return;
// }

static std::vector<Vector3> SupportContactFinder(const Vector3 & COMPos, const PIPInfo & PIPObj, const std::vector<Vector3> & ContactFreeContact, SignedDistanceFieldInfo & SDFInfo)
{
  // This function is used to get supportive contact and should be only called after ContactFreePoint function.
  // The introduced rotational momentum should be in the counter direction.
  Vector3 EdgeA = PIPObj.EdgeA;
  Vector3 EdgeB = PIPObj.EdgeB;
  Vector3 EdgeDir = EdgeB - EdgeA;
  std::vector<Vector3> SupportContact;
  SupportContact.reserve(ContactFreeContact.size());
  for (int i = 0; i < ContactFreeContact.size(); i++)
  {
    Vector3 ContactFreePoint = ContactFreeContact[i];
    Vector3 PointNormDir = SDFInfo.SignedDistanceNormal(ContactFreePoint);
    Vector3 rPos2COMPos = ContactFreePoint - COMPos;
    Vector3 InducedMomentum = cross(rPos2COMPos, PointNormDir);
    double ProjMomentumVal = InducedMomentum.x * EdgeDir.x + InducedMomentum.y * EdgeDir.y + InducedMomentum.z * EdgeDir.z;
    if(ProjMomentumVal<=0)
    {
      SupportContact.push_back(ContactFreePoint);
    }
  }
  return SupportContact;
}

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

static TrajInfo SingleContactPlanning(const PIPInfo & PIPObj, const int & LinkInfoIndex, const int & Type, const Robot & _SimRobot, const double & RefFailureMetric, const Vector3 & COMVel, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & _RobotContactInfo, ReachabilityMap & RMObject)
{
  /*
    This function is used to plan single contact: _RobotContactInfo keeps the robot presumably contact status.
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

  Vector3 RefContact;       // This is the position of the reference contact for robot's active end effector.
  SimRobot.GetWorldPosition(RobotLinkInfo[LinkInfoIndex].AvgLocalContact, RobotLinkInfo[LinkInfoIndex].LinkIndex, RefContact);
  std::vector<double> RefConfig(_SimRobot.q.size());
  for (int i = 0; i < _SimRobot.q.size(); i++)
  {
    RefConfig[i] =  _SimRobot.q[i];
  }
  std::vector<std::pair<Vector3, double>> ContactFreeInfo = ContactFreeInfoFn(SimRobot, RobotLinkInfo, OriRobotContactInfo, RMObject);

  // The following three are used to plot the all, active, and optimal
  std::vector<Vector3> IdealReachableContact = RMObject.IdealReachablePointsFinder(SimRobot, LinkInfoIndex);
  Vector3Writer(IdealReachableContact, "IdealReachableContact");

  clock_t beginTime = std::clock();
  std::vector<Vector3> ActiveReachableContact = RMObject.ReachablePointsFinder(SimRobot, LinkInfoIndex, NonlinearOptimizerInfo::SDFInfo);
  clock_t endTime = std::clock();
  double elapsed_secs = double(endTime - beginTime)/CLOCKS_PER_SEC;
  std::printf("ReachablePointsFinder function takes: %f ms\n", 1000.0 * elapsed_secs);


  beginTime = std::clock();
  std::vector<Vector3> ContactFreeContact = RMObject.ContactFreePointsFinder(RMObject.EndEffectorCollisionRadius[LinkInfoIndex], ActiveReachableContact, ContactFreeInfo);
  endTime = std::clock();
  elapsed_secs = double(endTime - beginTime)/CLOCKS_PER_SEC;
  std::printf("ContactFreeContact function takes: %f ms\n", 1000.0 * elapsed_secs);

  // Here we can make use of the PIPInfo to get rid of the extra contact points
  // The purpose is to select the contact point such that the rotational momentum can be reduced in to opposite direction.
  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(SimRobot, COMPos, COMVel);

  beginTime = std::clock();
  std::vector<Vector3> SupportContact = SupportContactFinder(COMPos, PIPObj, ContactFreeContact, NonlinearOptimizerInfo::SDFInfo);
  endTime = std::clock();
  elapsed_secs = double(endTime - beginTime)/CLOCKS_PER_SEC;
  std::printf("SupportContactFinder function takes: %f ms\n", 1000.0 * elapsed_secs);

  std::vector<ContactConfigInfo> SafeContact, BetterContact;
  SafeContact.reserve(SupportContact.size());
  BetterContact.reserve(SupportContact.size());
  beginTime = std::clock();

  std::vector<Vector3> SafeContactPos, BetterContactPos;
  FailureMetricEstimator(SupportContact, SafeContactPos, BetterContactPos, FixedContacts, COMPos, COMVel, RefContact, RefFailureMetric);
  // FailureMetricSelector(SimRobot, LinkInfoIndex, SupportContact, SafeContact, BetterContact, FixedContacts, COMPos, COMVel, RobotLinkInfo, RefFailureMetric, RMObject);
  endTime = std::clock();
  elapsed_secs = double(endTime - beginTime)/CLOCKS_PER_SEC;
  std::printf("FailureMetricEstimator function takes: %f ms\n", 1000.0 * elapsed_secs);

  Vector3Writer(ActiveReachableContact, "ActiveReachableContact");
  Vector3Writer(ContactFreeContact, "ContactFreeContact");
  Vector3Writer(SupportContact, "SupportContact");
  Vector3Writer(SafeContactPos, "SafeContact");
  Vector3Writer(BetterContactPos, "BetterContact");
  // Vector3Writer(OptimalContactPos, "OptimalContact");

  // Then the job is to generate transition Trajectory for robot's end effector.
  std::random_shuffle(SafeContactPos.begin(), SafeContactPos.end());

  // Here a number of transition trajectories can be generated.
  // Only one trajectory is needed actually.

  int FeasiFlag = 0;
  TrajInfo TransientTraj;
  for (int i = 0; i < SafeContactPos.size(); i++)
  {
    std::vector<double>   GoalConfig;
    std::vector<Vector3>  GoalContacts;
    int OptRes = ContactFeasibleOptFn(SimRobot, LinkInfoIndex, SafeContactPos[i], RobotLinkInfo, RMObject, GoalConfig, GoalContacts);
    beginTime = std::clock();
    TransientTraj = TransientTrajGene(SimRobot, LinkInfoIndex, RobotLinkInfo, RefConfig, RefContact, GoalConfig, SafeContactPos[0], RMObject, FeasiFlag);
    endTime = std::clock();
    elapsed_secs = double(endTime - beginTime)/CLOCKS_PER_SEC;
    std::printf("TransientTrajGene function takes: %f ms\n", 1000.0 * elapsed_secs);
    switch (FeasiFlag)
    {
      case 1:
      {
        return TransientTraj;
      }
      break;
      default:
      {

      }
      break;
    }
  }
  return TransientTraj;
}
static std::vector<TrajInfo> TransitionTrajGenerator(const PIPInfo & PIPObj, const std::vector<int> & ContactModiInfoIndices, const std::vector<int> & ContactAddInfoIndices, const Robot & SimRobot, const double & RefFailureMetric, const Vector3 & COMVel, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject)
{
  std::vector<TrajInfo> TransitionTrajs;
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
        TrajInfo TransitionTraj = SingleContactPlanning(PIPObj, ContactModiInfoIndices[i], 0, SimRobot, RefFailureMetric, COMVel, RobotLinkInfo, RobotContactInfo, RMObject);
        TransitionTrajs.push_back(TransitionTraj);
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
        TrajInfo TransitionTraj = SingleContactPlanning(PIPObj, ContactAddInfoIndices[i], 1, SimRobot, RefFailureMetric, COMVel, RobotLinkInfo, RobotContactInfo, RMObject);
        TransitionTrajs.push_back(TransitionTraj);
      }
    }
    break;
  }
  return TransitionTrajs;
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
  std::vector<TrajInfo> TransitionTrajs = TransitionTrajGenerator(PIPObj, ContactModiInfoIndices, ContactAddInfoIndices, SimRobot, RefFailureMetric, COMVel, RobotLinkInfo, RobotContactInfo, RMObject);
  int a = 1;
  return 1;
}
