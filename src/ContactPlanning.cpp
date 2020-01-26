#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <ctime>
#include <algorithm>
#include <random>
#include <queue>

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

    bool FeasiFlag;
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
          SplineObj = TransientTrajGene(SimRobot, LinkInfoIndex, RobotLinkInfo, RefContact, OptimalContact[i], RMObject, FeasiFlag);
          endTime = std::clock();
          elapsed_secs = double(endTime - beginTime)/CLOCKS_PER_SEC;
          std::printf("TransientTrajGene function takes: %f ms\n", 1000.0 * elapsed_secs);
          switch (FeasiFlag)
          {
            case true:
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

static std::vector<Vector3> OptimalContactSearcher(Robot & SimRobot, const PIPInfo & PIPObj, ReachabilityMap & RMObject, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & FixedRobotContactInfo, const int & SwingLimbIndex, const double & RefFailureMetric)
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
  std::vector<Vector3> ActiveReachableContact = RMObject.ReachablePointsFinder(SimRobot, SwingLimbIndex, NonlinearOptimizerInfo::SDFInfo);
  switch (ActiveReachableContact.size()){
    case 0:return OptimalContact;
    break;
    default:
    break;
  }

  // 1. Self-collision from other end effectors
  std::vector<Vector3> ContactFreeContact = RMObject.ContactFreePointsFinder(RMObject.EndEffectorCollisionRadius[SwingLimbIndex], ActiveReachableContact, ContactFreeInfo);
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

static double MinimumTimeEstimation(Robot & SimRobot, std::vector<int> & SwingLimbChain, const Config & qInit, const Config & qGoal)
{
  std::vector<double> ExecutationTime(SwingLimbChain.size());
  for (int i = 0; i < SwingLimbChain.size(); i++)
  {
    double ConfigDiff = qGoal[SwingLimbChain[i]] - qInit[SwingLimbChain[i]];
    ExecutationTime[i] = abs(ConfigDiff/SimRobot.velMax(SwingLimbChain[i]));
  }
  return *std::max_element(ExecutationTime.begin(), ExecutationTime.end());
}

static bool operator<(std::pair<Vector3, double> & a, std::pair<Vector3, double> & b)
{
  if( a.second> b.second)
  return false;
  return true;
}

static ControlReferenceInfo ControlReferenceGenerationInner(const Robot & _SimRobot, const PIPInfo & PIPObj, ReachabilityMap & RMObject, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & FixedRobotContactInfo, const int & SwingLimbIndex, const double & RefFailureMetric)
{
  // Here each planning I can at most give 100ms!

  Robot SimRobot = _SimRobot;

  ControlReferenceInfo ControlReferenceObj;

  Vector3 ContactInit;       // This is the position of the reference contact for robot's active end effector.
  SimRobot.GetWorldPosition(RobotLinkInfo[SwingLimbIndex].AvgLocalContact, RobotLinkInfo[SwingLimbIndex].LinkIndex, ContactInit);

  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(SimRobot, COMPos, COMVel);

  // clock_t beginTime = std::clock();
  std::vector<Vector3> OptimalContact = OptimalContactSearcher(SimRobot, PIPObj, RMObject, RobotLinkInfo, FixedRobotContactInfo, SwingLimbIndex, RefFailureMetric);
  // clock_t endTime = std::clock();
  // double elapsed_secs = double(endTime - beginTime)/CLOCKS_PER_SEC;
  // std::printf("OptimalContactSearcher function takes: %f ms\n", 1000.0 * elapsed_secs);

  switch (OptimalContact.size())
  {
    case 0:
    {
      return ControlReferenceObj;
    }
    break;
    default:
    {
      // Select the contact point such that COM projection has the highest distance to edge.
      std::vector<Vector3> SPVertices;
      for (int i = 0; i < RobotLinkInfo.size(); i++)
      {
        int LinkiPNo = RobotLinkInfo[i].LocalContacts.size();
        for (int j = 0; j < LinkiPNo; j++)
        {
          switch (FixedRobotContactInfo[i].LocalContactStatus[j])
          {
            case 0:
            break;
            case 1:
            {
              Vector3 LinkiPjPos;
              SimRobot.GetWorldPosition(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, LinkiPjPos);
              LinkiPjPos.z = 0.0;
              SPVertices.push_back(LinkiPjPos);
            }
            break;
            default:
            break;
          }
        }
      }
      Vector3 COM_Pos = SimRobot.GetCOM();
      int FacetFlag = 0;

      std::priority_queue<std::pair<Vector3, double>, std::vector<std::pair<Vector3, double>>, less<std::pair<Vector3, double>> > OptimalContactQueue;

      for (int i = 0; i < OptimalContact.size(); i++)
      {
        // Vector3 ContactCompare = OptimalContact[i] - ContactInit;
        // double ContactDis = ContactCompare.x * ContactCompare.x + ContactCompare.y * ContactCompare.y + ContactCompare.z * ContactCompare.z;
        std::vector<Vector3> NewSPVertices = SPVertices;
        NewSPVertices.push_back(OptimalContact[i]);
        FacetInfo SPObj = FlatContactHullGeneration(NewSPVertices, FacetFlag);    // This is the support polygon
        COM_Pos.z = 0.0;
        double COMDist = SPObj.ProjPoint2EdgeDist(COM_Pos);
        std::pair<Vector3, double> ContactPointPair(OptimalContact[i], COMDist);
        OptimalContactQueue.push(ContactPointPair);
      }

      std::pair<Vector3, double> ContactPair;

      // Now too early to assume that FailureFlag is true.
      bool FeasiFlag;
      std::vector<SplineLib::cSpline3> SplineObj;
      while(OptimalContactQueue.size()>0)
      {
        ContactPair = OptimalContactQueue.top();
        OptimalContactQueue.pop();
        Vector3 ContactGoal = ContactPair.first;
        SplineObj = TransientTrajGene(SimRobot, SwingLimbIndex, RobotLinkInfo, ContactInit, ContactGoal, RMObject, FeasiFlag);
        switch (FeasiFlag)
        {
          case true:
          {
            EndPathInfo EndPathObj(SplineObj, SwingLimbIndex);
            InvertedPendulumInfo InvertedPendulumObj(PIPObj.theta, PIPObj.thetadot, COMPos, COMVel);

            /*
              1. At each sampled waypoints along the end effector trajectory, an end effector position is evaluated from path.
              2. Based on robot's current configuration, an IK problem is solved to get robot's swing limb configuration.
              3. A time-optimal executation duration is computed.
              4. Based on that time, robot's whole-body configuration is updated with inverted pendulum model.
              5. The whole algorithm terminates when robot's self-collision has been triggered or no feasible IK solution can be found.
            */
            const int sNumber = 6;                 // 10 sampled points will be extracted from EndPathObj.
            int sIndex = 1;
            double sDiff = 1.0/(1.0 * sNumber - 1.0);
            double sVal = 0.0;
            Config CurrentConfig = SimRobot.q;
            double CurrentTime = 0.0;
            Vector3 CurrentContactPos = ContactInit;

            std::vector<Config> ConfigTraj;
            std::vector<double> TimeTraj;
            std::vector<Vector3> SwingLimbTraj;

            ConfigTraj.reserve(sNumber);
            TimeTraj.reserve(sNumber);
            SwingLimbTraj.reserve(sNumber);

            ConfigTraj.push_back(CurrentConfig);
            TimeTraj.push_back(CurrentTime);
            SwingLimbTraj.push_back(CurrentContactPos);

            bool OptFlag = true;
            while((sIndex<sNumber)&&(OptFlag == true))
            {
              sVal = 1.0 * sIndex * sDiff;
              EndPathObj.s2Pos(sVal, CurrentContactPos);
              std::vector<double> OptConfig = TransientOptFn(SimRobot, SwingLimbIndex, CurrentContactPos, RMObject, OptFlag);
              switch (OptFlag)
              {
                case false:
                {
                  // Stop testing the current ContactGoal and try a different one!
                  break;
                }
                break;
                default:
                {
                  // Minimum Time Estimation.
                  double CurrentTime_i = MinimumTimeEstimation(SimRobot, RMObject.EndEffectorLink2Pivotal[SwingLimbIndex], CurrentConfig, Config(OptConfig));

                  CurrentTime+=CurrentTime_i;

                  ConfigTraj.push_back(Config(OptConfig));
                  TimeTraj.push_back(CurrentTime);
                  SwingLimbTraj.push_back(CurrentContactPos);

                  // Then we should update the robot's CurrentConfig based on CurrentTime_i.
                  Config UpdatedConfig  = WholeBodyDynamicsIntegrator(SimRobot, OptConfig, PIPObj, InvertedPendulumObj, CurrentTime_i, sIndex);
                  SimRobot.UpdateConfig(UpdatedConfig);
                }
                break;
              }
              sIndex++;
            }
            // Here the inner optimiztion loop has been finished!
            switch (OptFlag)
            {
              case true:
              {
                /* This means that the current trajectories have been tested to be valid!
                        std::vector<Config> ConfigTraj;
                        std::vector<double> TimeTraj;
                        std::vector<Vector3> SwingLimbTraj;
                */
                // Here I would like to introduce another configuration to make sure that the contact can be firmly estabilished at the end.
                // std::vector<double> FinalRobotConfig = ContactFinalConfigOptimization(SimRobot, SwingLimbIndex, FixedRobotContactInfo, ConfigTraj[ConfigTraj.size()-1]);

                // std::string ConfigPath = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/user/hrp2/";
                // string _OptConfigFile = "OptConfig" + std::to_string(sNumber) + ".config";
                // RobotConfigWriter(FinalRobotConfig, ConfigPath, _OptConfigFile);

                ControlReferenceObj.TrajectoryUpdate(ConfigTraj, TimeTraj, SwingLimbTraj);
                // Then we need to estimate robot's impulse based on centroidal velocity.
                CollisionImpulseFunc(SimRobot, FixedRobotContactInfo, SwingLimbIndex, ControlReferenceObj);
                return ControlReferenceObj;
              }
              break;
              default:
              break;
            }
          }
          break;
          default:
          break;
        }
      }
    }
    break;
  }
  return ControlReferenceObj;
}

ControlReferenceInfo ControlReferenceGeneration(Robot & SimRobot, const PIPInfo & _PIPObj, const double & RefFailureMetric, const std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject, const double & TimeStep)
{
  // The whole planning algorithm should be written here.
  // The high-level idea is to plan individual end effector's configuration trajectory.
  // To avoid single leg jumping motion, we would like to exclude case where foot contact does not show up.

  AllContactStatusInfo AllContactStatusObj = SwingLimbIndices(SimRobot, RobotContactInfo);

  std::clock_t start_time;
  double allowd_time, duration_time;
  allowd_time = 0.25;                 // 250ms

  start_time = std::clock();          // get current time
  int ContactStatusOption = 0;
  std::vector<ControlReferenceInfo> RobotTrajVec;
  ControlReferenceInfo RobotTraj;
  std::vector<double> ImpulseVec;
  while((duration_time < allowd_time) && (ContactStatusOption < AllContactStatusObj.ContactStatusInfoVec.size()))
  {
    std::vector<ContactStatusInfo> RobotContactInfo = AllContactStatusObj.ContactStatusInfoVec[ContactStatusOption];
    int SwingLimbIndex = AllContactStatusObj.SwingLimbIndices[ContactStatusOption];

    Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
    CentroidalState(SimRobot, COMPos, COMVel);
    std::vector<Vector3> ActContactPos = ContactPositionFinder(SimRobot, NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfo);
    std::vector<PIPInfo> PIPTotal = PIPGenerator(ActContactPos, COMPos, COMVel);

    int PIPIndex;
    double RefFailureMetric = CapturePointGenerator(PIPTotal, PIPIndex);
    RobotTraj = ControlReferenceGenerationInner(SimRobot, PIPTotal[PIPIndex], RMObject, NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfo, SwingLimbIndex, RefFailureMetric);
    RobotTraj.SwingLimbIndex = SwingLimbIndex;
    duration_time = (std::clock() - start_time)/(double)CLOCKS_PER_SEC;
    std::printf("Planning takes: %f ms\n", 1000.0 * duration_time);
    start_time = std::clock();          // get current time

    switch (RobotTraj.ControlReferenceFlag)
    {
      case true:
      {
        RobotTrajVec.push_back(RobotTraj);
        ImpulseVec.push_back(RobotTraj.Impulse);
      }
      break;
      default:
      break;
    }
    ContactStatusOption++;
  }
  // Based on the value of the impulse, let's select the one with the lowest impulse.
  switch (RobotTrajVec.size())
  {
    case 0:
    {
      std::printf("Planning fails to find a feasible solution! \n");
      // Planning fails to find a feasible solution!
      return RobotTraj;
    }
    break;
    default:
    {
      int RobotTrajIndex = std::distance(ImpulseVec.begin(), std::min_element(ImpulseVec.begin(), ImpulseVec.end()));
      std::printf("Planning successfully finds a feasible solution! \nRobot Limb Index: %d\n", RobotTrajVec[RobotTrajIndex].SwingLimbIndex);
      RobotTraj = RobotTrajVec[RobotTrajIndex];
    }
    break;
  }
  return RobotTraj;
}

double PresumeContactMinDis(Robot & SimRobot, const std::vector<ContactStatusInfo> & RobotContactInfo)
{
  // This function calculates the maximum distance of minimum end effectors.

  double Dis = 0.0;
  double ContactDist;
  std::vector<double> ContactDisVec;
  for (int i = 0; i < RobotContactInfo.size(); i++)
  {
    switch (RobotContactInfo[i].LocalContactStatus[0])
    {
      case 1:
      {
        ContactDist = 100000.0;
        for(int j = 0; j < NonlinearOptimizerInfo::RobotLinkInfo[i].LocalContacts.size(); j++)
        {
          Vector3 ContactPos;
          SimRobot.GetWorldPosition(NonlinearOptimizerInfo::RobotLinkInfo[i].LocalContacts[j], RobotContactInfo[i].LinkIndex, ContactPos);
          double RefPosDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(ContactPos);
          if(ContactDist>RefPosDist)
          {
            ContactDist = RefPosDist;
          }
        }
      }
      break;
      default:
      ContactDist = 0.0;
      break;
    }
    ContactDisVec.push_back(ContactDist);
  }
  return *max_element(ContactDisVec.begin(), ContactDisVec.end());
}
