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

static std::vector<Vector3> OptimalContactFinder(const std::vector<Vector3> & SupportContact, const std::vector<Vector3> & FixedContacts, const Vector3 & COMPos, const Vector3 & COMVel, const double & RefFailureMetric)
{
  // This function is used to estimate robot's cost based on the point contact addition for failure metric evaluation.
  std::vector<Vector3> OptimalContact;
  OptimalContact.reserve(SupportContact.size());
  std::vector<double> ContactFailureMetric(SupportContact.size());

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
    ContactFailureMetric[i] = CPObjective;
    if(CPObjective<RefFailureMetric)
    {
      // This indicates that the curent contact is at least a better contact.
      if(CPObjective<Tol)
      {
        // This means that current contact is OptimalContact
        OptimalContact.push_back(SupportContact[i]);
      }
    }
  }

  switch (OptimalContact.size())
  {
    case 0:
    {
      // This indicates the OptimalContact does not have any element.
      // Then the idea is to get the lowest cost from failure metric as the optimal contact.
      int OptiIndex = std::distance(ContactFailureMetric.begin(), std::min_element(ContactFailureMetric.begin(), ContactFailureMetric.end()));
      OptimalContact.push_back(SupportContact[OptiIndex]);
    }
    break;
    default:
    break;
  }
  return OptimalContact;
}

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

static EndPathInfo SingleContactPlanning(const PIPInfo & PIPObj, const int & LinkInfoIndex, const int & Type, const Robot & _SimRobot, const double & RefFailureMetric, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & _RobotContactInfo, ReachabilityMap & RMObject)
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

    beginTime = std::clock();
    std::vector<Vector3> OptimalContact = OptimalContactFinder(SupportContact, FixedContacts, COMPos, COMVel, RefFailureMetric);
    endTime = std::clock();
    elapsed_secs = double(endTime - beginTime)/CLOCKS_PER_SEC;
    std::printf("OptimalContactFinder function takes: %f ms\n", 1000.0 * elapsed_secs);

    Vector3Writer(ActiveReachableContact, "ActiveReachableContact");
    Vector3Writer(ContactFreeContact, "ContactFreeContact");
    Vector3Writer(SupportContact, "SupportContact");
    Vector3Writer(OptimalContact, "OptimalContact");

    // Then the job is to generate transition Trajectory for robot's end effector.
    std::random_shuffle(OptimalContact.begin(), OptimalContact.end());

    // Here a number of transition trajectories can be generated.
    // Only one trajectory is needed actually.

    int FeasiFlag = 0;
    std::vector<SplineLib::cSpline3> SplineObj;
    for (int i = 0; i < OptimalContact.size(); i++)
    {
      std::vector<double>   GoalConfig;
      std::vector<Vector3>  GoalContacts;
      int OptRes = ContactFeasibleOptFn(SimRobot, LinkInfoIndex, OptimalContact[i], RobotLinkInfo, RMObject, GoalConfig, GoalContacts);
      switch (OptRes)
      {
        case 1:
        {
          beginTime = std::clock();
          SplineObj = TransientTrajGene(SimRobot, LinkInfoIndex, RobotLinkInfo, RefConfig, RefContact, OptimalContact[i], RMObject, FeasiFlag);
          endTime = std::clock();
          elapsed_secs = double(endTime - beginTime)/CLOCKS_PER_SEC;
          std::printf("TransientTrajGene function takes: %f ms\n", 1000.0 * elapsed_secs);
          switch (FeasiFlag)
          {
            case 1:
            {
              return EndPathInfo(SplineObj, LinkInfoIndex);
            }
            break;
            default:
            {

            }
            break;
          }
        }
        break;
        default:
        break;
      }
    }
    return EndPathInfo(SplineObj, LinkInfoIndex);
  }

EndPathInfo EndEffectorPlanner(Robot & SimRobot, const PIPInfo & PIPObj, const double & RefFailureMetric, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject, const double & TimeStep)
{
  std::vector<Vector3> ModiCOMPosTraj, ModiCOMVelTraj;
  int FixedContactIndex;
  double ContactModiColTime = ContactModiPreEstimation(SimRobot, PIPObj, RobotLinkInfo, RobotContactInfo, NonlinearOptimizerInfo::SDFInfo, FixedContactIndex, ModiCOMPosTraj, ModiCOMVelTraj, TimeStep);
  std::vector<int> ContactModiInfoIndices;
  for (int i = 0; i < RobotLinkInfo.size(); i++){
    switch (RobotContactInfo[i].LocalContactStatus[0]){
      case 1: if(i != FixedContactIndex) ContactModiInfoIndices.push_back(i);
      break;
      default:
      break;
    }
  }
  /*
    Control Strategy: Using Inverted Pendulum for Centroidal Dynamics Estimation and Compute Control Law
  */
  EndPathInfo TransitionTraj = SingleContactPlanning(PIPObj, ContactModiInfoIndices[0], 0, SimRobot, RefFailureMetric, RobotLinkInfo, RobotContactInfo, RMObject);
  return TransitionTraj;
}

static AllContactStatusInfo SwingLimbIndices(Robot & SimRobot, const std::vector<ContactStatusInfo> & RobotContactInfo)
{
  // This function is used to generate Swing Limb Indices and its associated contact status info.
  // First  part is for contact modification.
  // Second part is for contact addition.

  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(SimRobot, COMPos, COMVel);

  AllContactStatusInfo AllContactStatusObj;
  int LegActNo = RobotContactInfo[0].LocalContactStatus[0] + RobotContactInfo[1].LocalContactStatus[0];
  int AllActNo = LegActNo + RobotContactInfo[2].LocalContactStatus[0] + RobotContactInfo[3].LocalContactStatus[0];

  // Contact Modification
  switch (AllActNo)
  {
    case 0:
    {
      std::cerr<<"No Active Contact!"<<endl;
      exit(1);
    }
    break;
    case 1:
      // No contact modification
    break;
    default:
    {
      // Algorithms for contact modification
      switch (LegActNo)
      {
        case 0:
        {
          std::cerr<<"No Active Foot Contact!"<<endl;
          exit(1);
        }
        break;
        case 1:
        {
          switch (RobotContactInfo[2].LocalContactStatus[0] + RobotContactInfo[3].LocalContactStatus[0])
          {
            case 2:
            {
              std::vector<ContactStatusInfo> RobotContactInfoFirst = RobotContactInfo;
              RobotContactInfoFirst[2].StatusSwitch(0);
              double FMFirst = FailureMetricEval(SimRobot, NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfoFirst);

              std::vector<ContactStatusInfo> RobotContactInfoSecond = RobotContactInfo;
              RobotContactInfoSecond[3].StatusSwitch(0);
              double FMSecond = FailureMetricEval(SimRobot, NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfoSecond);
              if(FMFirst>FMSecond)
              {
                // We should move second contact.
                AllContactStatusObj.ContactStatusAppender(RobotContactInfoSecond, 3);
              }
              else
              {
                // We should move first contact point.
                AllContactStatusObj.ContactStatusAppender(RobotContactInfoFirst, 2);
              }
            }
            break;
            default:
            {
              switch (RobotContactInfo[2].LocalContactStatus[0])
              {
                case 1:
                {
                  std::vector<ContactStatusInfo> RobotContactInfoTemp = RobotContactInfo;
                  RobotContactInfoTemp[2].StatusSwitch(0);
                  AllContactStatusObj.ContactStatusAppender(RobotContactInfoTemp, 2);
                }
                break;
                default:
                break;
              }
              switch (RobotContactInfo[3].LocalContactStatus[0])
              {
                case 1:
                {
                  std::vector<ContactStatusInfo> RobotContactInfoTemp = RobotContactInfo;
                  RobotContactInfoTemp[3].StatusSwitch(0);
                  AllContactStatusObj.ContactStatusAppender(RobotContactInfoTemp, 3);
                }
                break;
                default:
                break;
              }
            }
            break;
          }
        }
        break;
        default:
        {
          // More general case where two feet contact is shown while hands may be involved.
          std::vector<double> FMVec;
          std::vector<int> IndicesVec;
          for(int i = 0; i < RobotContactInfo.size(); i++)
          {
            switch (RobotContactInfo[i].LocalContactStatus[0])
            {
              case 1:
              {
                std::vector<ContactStatusInfo> RobotContactInfoTemp = RobotContactInfo;
                RobotContactInfoTemp[i].StatusSwitch(0);
                double FMi = FailureMetricEval(SimRobot, NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfoTemp);
                FMVec.push_back(FMi);
                IndicesVec.push_back(i);
              }
              break;
            }
          }
          int MaxEndIndex = std::distance(FMVec.begin(), std::max_element(FMVec.begin(), FMVec.end()));
          IndicesVec.erase(IndicesVec.begin() + MaxEndIndex);
          for (int i = 0; i < IndicesVec.size(); i++)
          {
            std::vector<ContactStatusInfo> RobotContactInfoTemp = RobotContactInfo;
            RobotContactInfoTemp[IndicesVec[i]].StatusSwitch(0);
            AllContactStatusObj.ContactStatusAppender(RobotContactInfoTemp, IndicesVec[i]);
          }
        }
        break;
      }
    }
    break;
  }

  // Contact Addition
  for (int i = 0; i < RobotContactInfo.size(); i++)
  {
    switch (RobotContactInfo[i].LocalContactStatus[0])
    {
      case 0:
      {
        std::vector<ContactStatusInfo> RobotContactInfoTemp = RobotContactInfo;
        AllContactStatusObj.ContactStatusAppender(RobotContactInfoTemp, i);
      }
      break;
    }
  }
  return AllContactStatusObj;
}

static std::vector<Vector3> OptimalContactSearcher(Robot & SimRobot, const PIPInfo & PIPObj, ReachabilityMap & RMObject, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & FixedRobotContactInfo, const int & SwingLinkIndex, const double & RefFailureMetric)
{
  std::vector<Vector3> OptimalContact;
  // Get all the fixed contact positions.
  std::vector<Vector3> FixedContactPos;
  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    for (int j = 0; j < RobotLinkInfo[i].LocalContacts.size(); j++)
    {
      switch (FixedRobotContactInfo[i].LocalContactStatus[j])
      {
        case 1:
        {
          // This means that current contact is active and we should keep its location and Jacobian.
          Vector3 LinkiPjPos;
          SimRobot.GetWorldPosition(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, LinkiPjPos);
          FixedContactPos.push_back(LinkiPjPos);
        }
        break;
        default:
        break;
      }
    }
  }
  std::vector<std::pair<Vector3, double>> ContactFreeInfo = ContactFreeInfoFn(SimRobot, RobotLinkInfo, FixedRobotContactInfo, RMObject);

  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(SimRobot, COMPos, COMVel);

  /*
    Data point filtering
  */
  //  0. Reachable with respect to the pivotal joint
  std::vector<Vector3> ActiveReachableContact = RMObject.ReachablePointsFinder(SimRobot, SwingLinkIndex, NonlinearOptimizerInfo::SDFInfo);
  switch (ActiveReachableContact.size()){
    case 0:return OptimalContact;
    break;
    default:
    break;
  }

  // 1. Self-collision from other end effectors
  std::vector<Vector3> ContactFreeContact = RMObject.ContactFreePointsFinder(RMObject.EndEffectorCollisionRadius[SwingLinkIndex], ActiveReachableContact, ContactFreeInfo);
  switch (ContactFreeContact.size()){
    case 0:return OptimalContact;
    break;
    default:
    break;
  }
  // 2. Supportive
  std::vector<Vector3> SupportContact = SupportContactFinder(COMPos, PIPObj, ContactFreeContact, NonlinearOptimizerInfo::SDFInfo);
  switch (SupportContact.size()){
    case 0:return OptimalContact;
    break;
    default:
    break;
  }
  // 3. Optimal Contact
  OptimalContact = OptimalContactFinder(SupportContact, FixedContactPos, COMPos, COMVel, RefFailureMetric);

  Vector3Writer(ActiveReachableContact, "ActiveReachableContact");
  Vector3Writer(ContactFreeContact, "ContactFreeContact");
  Vector3Writer(SupportContact, "SupportContact");
  Vector3Writer(OptimalContact, "OptimalContact");

  return OptimalContact;
}

ControlReferenceInfo ControlReferenceGenerationInner(Robot & SimRobot, const PIPInfo & PIPObj, ReachabilityMap & RMObject, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & FixedRobotContactInfo, const int & SwingLinkIndex, const double & RefFailureMetric, double & Impulse, int & FailureFlag)
{
  // Here each planning I can at most give 100ms!

  ControlReferenceInfo ControlReferenceObj;

  Vector3 RefContact;       // This is the position of the reference contact for robot's active end effector.
  SimRobot.GetWorldPosition(RobotLinkInfo[SwingLinkIndex].AvgLocalContact, RobotLinkInfo[SwingLinkIndex].LinkIndex, RefContact);
  FailureFlag = 0;

  clock_t beginTime = std::clock();
  std::vector<Vector3> OptimalContact = OptimalContactSearcher(SimRobot, PIPObj, RMObject, RobotLinkInfo, FixedRobotContactInfo, SwingLinkIndex, RefFailureMetric);
  clock_t endTime = std::clock();
  double elapsed_secs = double(endTime - beginTime)/CLOCKS_PER_SEC;
  // std::printf("OptimalContactSearcher function takes: %f ms\n", 1000.0 * elapsed_secs);

  switch (OptimalContact.size())
  {
    case 0:
    {
      FailureFlag = 1;
      return ControlReferenceObj;
    }
    break;
    default:
    {
      FailureFlag = 0;
      std::random_shuffle(OptimalContact.begin(), OptimalContact.end());

    }
    break;
  }
}

ControlReferenceInfo ControlReferenceGeneration(Robot & SimRobot, const PIPInfo & PIPObj, const double & RefFailureMetric, const std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject, const double & TimeStep)
{
  // The whole planning algorithm should be written here.
  // The high-level idea is to plan individual end effector's configuration trajectory.
  // To avoid single leg jumping motion, we would like to exclude case where foot contact does not show up.

  AllContactStatusInfo AllContactStatusObj = SwingLimbIndices(SimRobot, RobotContactInfo);

  std::clock_t start_time;
  double allowd_time, duration_time;
  allowd_time = 0.25;                 // 250ms

  start_time = std::clock();          // get current time
  duration_time = (std::clock() - start_time)/(double)CLOCKS_PER_SEC;

  int ContactStatusOption = 0;
  while((duration_time < allowd_time) && (ContactStatusOption < AllContactStatusObj.ContactStatusInfoVec.size()))
  {
    std::vector<ContactStatusInfo> RobotContactInfo = AllContactStatusObj.ContactStatusInfoVec[ContactStatusOption];
    int SwingLimbIndex = AllContactStatusObj.SwingLimbIndices[ContactStatusOption];
    double Impulse = 0.0;
    int FailureFlag = 0;
    ControlReferenceInfo RobotTraj = ControlReferenceGenerationInner(SimRobot, PIPObj, RMObject, NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfo, SwingLimbIndex, RefFailureMetric, Impulse, FailureFlag);
    duration_time = (std::clock() - start_time)/(double)CLOCKS_PER_SEC;
  }

}
