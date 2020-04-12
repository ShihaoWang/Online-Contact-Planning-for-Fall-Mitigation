#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <ctime>
#include <algorithm>
#include <random>
#include <queue>
#include <math.h>

static bool ContactPairCMP(const pair<Vector3, double> & a, const pair<Vector3, double> & b)
{
    return (a.second > b.second);
}

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

  double Tol = 1e-5;

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
  // The rotation axis is the only thing needed here!
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

static AllContactStatusInfo SwingLimbIndices(Robot & SimRobot, const std::vector<ContactStatusInfo> & RobotContactInfo)
{
  // This function is used to generate Swing Limb Indices and its associated contact status info.
  // First part is for contact modification.
  // Second part is for contact addition.

  // This part should be rewritten to allow more general case.

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
          // In this case, the contact modification can only be conducted for hand contact if there exists.
          switch (RobotContactInfo[2].LocalContactStatus[0])
          {
            case 1:
            {
              std::vector<ContactStatusInfo> RobotContactInfoModi = RobotContactInfo;
              RobotContactInfoModi[2].StatusSwitch(0);
              AllContactStatusObj.ContactStatusAppender(RobotContactInfoModi, 2);
            }
            break;
            default:
            break;
          }
          switch (RobotContactInfo[3].LocalContactStatus[0])
          {
            case 1:
            {
              std::vector<ContactStatusInfo> RobotContactInfoModi = RobotContactInfo;
              RobotContactInfoModi[3].StatusSwitch(0);
              AllContactStatusObj.ContactStatusAppender(RobotContactInfoModi, 3);
            }
            break;
            default:
            break;
          }
        }
        break;
        default:
        {
          // More general case where two feet contact is shown while hands may be involved.
          for (int i = 0; i < 4; i++)
          {
            switch (RobotContactInfo[i].LocalContactStatus[0])
            {
              case 1:
              {
                std::vector<ContactStatusInfo> RobotContactInfoModi = RobotContactInfo;
                RobotContactInfoModi[i].StatusSwitch(0);
                AllContactStatusObj.ContactStatusAppender(RobotContactInfoModi, i);
              }
              break;
              default:
              break;
            }
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

static std::vector<Vector3> OptimalContactSearcher(const Robot & SimRobot, const PIPInfo & PIPObj, const Vector3 & PredictedCOMPos, ReachabilityMap & RMObject, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & FixedRobotContactInfo, const int & SwingLimbIndex, const double & RefFailureMetric, DataRecorderInfo & DataRecorderObj)
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
  Vector3Writer(ActiveReachableContact, "ActiveReachableContact");

  // 1. Self-collision from other end effectors
  std::vector<Vector3> ContactFreeContact = RMObject.ContactFreePointsFinder(RMObject.EndEffectorCollisionRadius[SwingLimbIndex], ActiveReachableContact, ContactFreeInfo);
  switch (ContactFreeContact.size()){
    case 0:return OptimalContact;
    break;
    default:
    break;
  }
  Vector3Writer(ContactFreeContact, "ContactFreeContact");

  // 2. Supportive
  std::vector<Vector3> SupportContact = SupportContactFinder(COMPos, PIPObj, ContactFreeContact, NonlinearOptimizerInfo::SDFInfo);
  switch (SupportContact.size()){
    case 0:return OptimalContact;
    break;
    default:
    break;
  }
  Vector3Writer(SupportContact, "SupportContact");

  // 3. Optimal Contact
  OptimalContact = OptimalContactFinder(SupportContact, FixedContactPos, COMPos, COMVel, RefFailureMetric);
  switch (OptimalContact.size())
  {
    case 0:
    {
      return OptimalContact;
    }
    break;
    default:
    break;
  }
  Vector3Writer(OptimalContact, "OptimalContact");

  const int CutOffNo = 10;

  // 4. Selected Optimal Contact

  /*    Method 1:   highest distance to edge*/
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

  Vector3 COM_Pos = PredictedCOMPos;
  std::vector<std::pair<Vector3, double>> ContactPairVec;
  ContactPairVec.reserve(OptimalContact.size());
  // Here a little modification will be made to ensure a more accurate computation of contact points.
  std::vector<Vector3> SwingLimbVertices;
  SwingLimbVertices.reserve(RobotLinkInfo[SwingLimbIndex].LocalContacts.size());
  for (int i = 0; i < RobotLinkInfo[SwingLimbIndex].LocalContacts.size(); i++)
  {
    Vector3 LinkiPjPos;
    SimRobot.GetWorldPosition(RobotLinkInfo[SwingLimbIndex].LocalContacts[i], RobotLinkInfo[SwingLimbIndex].LinkIndex, LinkiPjPos);
    SwingLimbVertices.push_back(LinkiPjPos);
  }

  Vector3 SwingLimbAvg;
  SimRobot.GetWorldPosition(RobotLinkInfo[SwingLimbIndex].AvgLocalContact, RobotLinkInfo[SwingLimbIndex].LinkIndex, SwingLimbAvg);

  int FacetFlag = 0;
  for (int i = 0; i < OptimalContact.size(); i++)
  {
    std::vector<Vector3> NewSPVertices = SPVertices;
    Vector3 ShiftVec = OptimalContact[i] - SwingLimbAvg;
    for (int j = 0; j < SwingLimbVertices.size(); j++)
    {
      NewSPVertices.push_back(SwingLimbVertices[j] + ShiftVec);
    }
    FacetInfo SPObj = FlatContactHullGeneration(NewSPVertices, FacetFlag);    // This is the support polygon
    COM_Pos.z = 0.0;
    double COMDist = SPObj.ProjPoint2EdgeDist(COM_Pos);
    std::pair<Vector3, double> ContactPair_i = std::make_pair(OptimalContact[i], COMDist) ;
    ContactPairVec.push_back(ContactPair_i);
  }

  /*    Method 2:  Random Selection */
  // std::vector<int> OptimalContactIndices(OptimalContact.size());
  // for (int i = 0; i < OptimalContact.size(); i++)
  // {
  //   OptimalContactIndices[i] = i;
  // }
  //
  // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  // std::default_random_engine e(seed);
  // std::shuffle(std::begin(OptimalContactIndices), std::end(OptimalContactIndices), e);
  // std::vector<Vector3> ReducedOptimalContact;
  // if(OptimalContactIndices.size()>CutOffNo)
  // {
  //   for (int i = 0; i < CutOffNo; i++)
  //   {
  //     ReducedOptimalContact.push_back(OptimalContact[OptimalContactIndices[i]]);
  //   }
  // }
  // else
  // {
  //   ReducedOptimalContact = OptimalContact;
  // }

  // /*    Method 3:  Momentum Projection */
  // std::vector<std::pair<Vector3, double>> ContactPairVec;
  // ContactPairVec.reserve(OptimalContact.size());
  // Vector3 RotAxis = PIPObj.x_unit;
  // Vector3 RefPoint = PIPObj.Intersection;
  // for (int i = 0; i < OptimalContact.size(); i++)
  // {
  //   Vector3 ContactForceNorm = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(OptimalContact[i]);
  //   Vector3 SupportMomentum = cross(ContactForceNorm, OptimalContact[i] - RefPoint);
  //   double SupportMomentumProj = RotAxis.dot(SupportMomentum);
  //   std::pair<Vector3, double> ContactPair_i = std::make_pair(OptimalContact[i], SupportMomentumProj) ;
  //   ContactPairVec.push_back(ContactPair_i);
  // }

  // /*    Method 4:  Centroidal Direction */
  // std::vector<std::pair<Vector3, double>> ContactPairVec;
  // ContactPairVec.reserve(OptimalContact.size());
  // for (int i = 0; i < OptimalContact.size(); i++)
  // {
  //   double SupportMomentumProj = COMVel.dot(OptimalContact[i] - COM_Pos);
  //   std::pair<Vector3, double> ContactPair_i = std::make_pair(OptimalContact[i], SupportMomentumProj) ;
  //   ContactPairVec.push_back(ContactPair_i);
  // }

  sort(ContactPairVec.begin(), ContactPairVec.end(), ContactPairCMP);
  std::vector<Vector3> ReducedOptimalContact;

  if(ContactPairVec.size()>CutOffNo)
  {
    for (int i = 0; i < CutOffNo; i++)
    {
      ReducedOptimalContact.push_back(ContactPairVec[i].first);
    }
  }
  else
  {
    for (int i = 0; i < ContactPairVec.size(); i++)
    {
      ReducedOptimalContact.push_back(ContactPairVec[i].first);
    }
  }

  // In The End we give all these values
  DataRecorderObj.ActiveReachableContact = ActiveReachableContact;
  DataRecorderObj.ContactFreeContact = ContactFreeContact;
  DataRecorderObj.SupportContact = SupportContact;
  DataRecorderObj.OptimalContact = OptimalContact;
  DataRecorderObj.ReducedOptimalContact = ReducedOptimalContact;
  Vector3Writer(ReducedOptimalContact, "ReducedOptimalContact");

  return ReducedOptimalContact;
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

static ControlReferenceInfo ControlReferenceGenerationInner(const Robot & SimRobot, const PIPInfo & PIPObj, const Vector3 & PredictedCOMPos, ReachabilityMap & RMObject, SelfLinkGeoInfo & SelfLinkGeoObj, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & FixedRobotContactInfo, const int & SwingLimbIndex, const double & RefFailureMetric, DataRecorderInfo & DataRecorderObj)
{
  ControlReferenceInfo ControlReferenceObj;
  Vector3 ContactInit;       // This is the position of the reference contact for robot's active end effector.
  SimRobot.GetWorldPosition(RobotLinkInfo[SwingLimbIndex].AvgLocalContact, RobotLinkInfo[SwingLimbIndex].LinkIndex, ContactInit);
  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(SimRobot, COMPos, COMVel);
  std::vector<Vector3> OptimalContact = OptimalContactSearcher(SimRobot, PIPObj, PredictedCOMPos, RMObject, RobotLinkInfo, FixedRobotContactInfo, SwingLimbIndex, RefFailureMetric, DataRecorderObj);
  switch (OptimalContact.size())
  {
    case 0:
    {
      return ControlReferenceObj;
    }
    break;
    default:
    {
      // Now too early to assume that FailureFlag is true.
      bool FeasiFlag;
      std::vector<SplineLib::cSpline3> SplineObj;
      int OptimalContactIndex = 0;
      while(OptimalContactIndex<OptimalContact.size())
      {
        Robot SimRobotInner = SimRobot;
        Vector3 ContactGoal = OptimalContact[OptimalContactIndex];
        Vector3 ContactGoalGrad = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(ContactGoal);
        SplineObj = TransientTrajGene(SimRobotInner, SwingLimbIndex, SelfLinkGeoObj, RobotLinkInfo, ContactInit, ContactGoal, RMObject, DataRecorderObj, FeasiFlag);
        if(FeasiFlag)
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
          const int sNumber = 6;                 // 6 sampled points will be extracted from EndPathObj.
          int sIndex = 1;
          double sDiff = 1.0/(1.0 * sNumber - 1.0);
          double sVal = 0.0;
          Config CurrentConfig = SimRobotInner.q;
          CurrentConfig = YPRShifter(CurrentConfig);
          double CurrentTime = 0.0;
          Vector3 CurrentContactPos = ContactInit;

          std::vector<double> TimeTraj;
          std::vector<Config> ConfigTraj;

          TimeTraj.reserve(sNumber);
          ConfigTraj.reserve(sNumber);

          TimeTraj.push_back(CurrentTime);
          ConfigTraj.push_back(CurrentConfig);

          bool OptFlag = true;
          while((sIndex<sNumber)&&(OptFlag == true))
          {
            sVal = 1.0 * sIndex * sDiff;
            EndPathObj.s2Pos(sVal, CurrentContactPos);
            bool LastFlag;
            switch (sIndex)
            {
              case 5:   LastFlag = true;
              break;
              default:  LastFlag = false;
              break;
            }
            std::vector<double> OptConfig = TransientOptFn(SimRobotInner, SwingLimbIndex, SelfLinkGeoObj, CurrentContactPos, RMObject, OptFlag, LastFlag);;
            if(OptFlag)
            {
              // Minimum Time Estimation.
              double CurrentTime_i = MinimumTimeEstimation(SimRobotInner, RMObject.EndEffectorLink2Pivotal[SwingLimbIndex], CurrentConfig, Config(OptConfig));

              CurrentTime+=CurrentTime_i;
              TimeTraj.push_back(CurrentTime);
              ConfigTraj.push_back(Config(OptConfig));

              // Then we should update the robot's CurrentConfig based on CurrentTime_i.
              Config UpdatedConfig  = WholeBodyDynamicsIntegrator(SimRobotInner, OptConfig, PIPObj, InvertedPendulumObj, CurrentTime_i, sIndex);
              SimRobotInner.UpdateConfig(UpdatedConfig);
              CurrentConfig = UpdatedConfig;
            }
            else
            {
              break;
            }
            sIndex++;
          }
          // Here the inner optimiztion loop has been finished!
          if(OptFlag)
          {
            ControlReferenceObj.TrajectoryUpdate(TimeTraj, ConfigTraj, ContactGoal, ContactGoalGrad);
            // Impulse is not going to be computed due to numerical infeasibility!
            ControlReferenceObj.Impulse = 0.0;
            // CollisionImpulseFunc(SimRobotInner, FixedRobotContactInfo, SwingLimbIndex, ControlReferenceObj);

            std::vector<ContactStatusInfo> GoalContactInfo = FixedRobotContactInfo;
            for(int i = 0; i<FixedRobotContactInfo[SwingLimbIndex].LocalContactStatus.size(); i++)
            {
              GoalContactInfo[SwingLimbIndex].LocalContactStatus[i] = 1;
            }
            ControlReferenceObj.InitContactInfo = FixedRobotContactInfo;
            ControlReferenceObj.GoalContactInfo = GoalContactInfo;

            DataRecorderObj.OptConfigs = ConfigTraj;

            return ControlReferenceObj;
          }
        }
        OptimalContactIndex++;
      }
    }
    break;
  }
  return ControlReferenceObj;
}

ControlReferenceInfo ControlReferenceGeneration(Robot & SimRobot, const Vector3 & COMPos, const Vector3 & COMVel, const double & RefFailureMetric, const std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject, SelfLinkGeoInfo & SelfLinkGeoObj, const double & TimeStep, double & PlanTime, const string & SpecificPath, const int & PlanningSteps, const double & DistTol, const int & ContactStatusOptionRef)
{
  // The whole planning algorithm should be written here.
  // The high-level idea is to plan individual end effector's configuration trajectory.

  AllContactStatusInfo AllContactStatusObj = SwingLimbIndices(SimRobot, RobotContactInfo);
  std::clock_t start_time = std::clock();          // get current time
  int ContactStatusOption = 0;
  int LimbSuccessNo = 0;
  std::vector<ControlReferenceInfo> RobotTrajVec;
  ControlReferenceInfo RobotTraj;
  std::vector<double> ExeTimeVec;
  std::vector<double> EndDistVec;

  for(ContactStatusOption = 0; ContactStatusOption< AllContactStatusObj.ContactStatusInfoVec.size(); ContactStatusOption++){
    if(ContactStatusOptionRef!=-1){
      if(ContactStatusOption!=ContactStatusOptionRef){
        continue;
      }
    }
    std::vector<ContactStatusInfo> RobotContactInfo = AllContactStatusObj.ContactStatusInfoVec[ContactStatusOption];
    int SwingLimbIndex = AllContactStatusObj.SwingLimbIndices[ContactStatusOption];
    std::vector<Vector3> ActContactPos = ContactPositionFinder(SimRobot, NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfo);    // From ContactInfoActive

    DataRecorderInfo DataRecorderObj;

    // Here TipOverPIP needs to be computed given RobotContactInfo!
    Vector3 PredictedCOMPos;
    PIPInfo TipOverPIP = TipOverPIPGene(ActContactPos, COMPos, COMVel, PredictedCOMPos);

    RobotTraj = ControlReferenceGenerationInner(SimRobot, TipOverPIP, PredictedCOMPos, RMObject, SelfLinkGeoObj, NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfo, SwingLimbIndex, RefFailureMetric, DataRecorderObj);
    RobotTraj.SwingLimbIndex = SwingLimbIndex;
    RobotTraj.ContactStatusOptionIndex = ContactStatusOption;
    double duration_time = (std::clock() - start_time)/(double)CLOCKS_PER_SEC;
    std::printf("Planning takes: %f ms\n", 1000.0 * duration_time);
    start_time = std::clock();          // get current time
    RobotTraj.PlanningTime = 1000.0 * duration_time;      // The unit is ms.
    PlanTime+= 1000.0 * duration_time;
    if(RobotTraj.ControlReferenceFlag)
    {
      DataRecorderObj.PlanningNo = PlanningSteps;
      DataRecorderObj.LimbNo = LimbSuccessNo;
      DataRecorderObj.DataRecorder(SpecificPath);
      for (int i = 0; i < DataRecorderObj.OptConfigs.size(); i++)
      {
        const string OptConfigFile = std::to_string(PlanningSteps) + "_" + std::to_string(LimbSuccessNo) + "_" + "OptConfig" + std::to_string(i) + ".config";
        RobotConfigWriter(DataRecorderObj.OptConfigs[i], SpecificPath, OptConfigFile);
      }
      LimbSuccessNo++;

      RobotTrajVec.push_back(RobotTraj);
      ExeTimeVec.push_back(RobotTraj.PlanStateTraj.EndTime());

      double EndDist = PresumeContactMinDis(SimRobot, RobotContactInfo);
      EndDistVec.push_back(EndDist);
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
      PlanningInfoFileAppender(PlanningSteps, RobotTrajVec.size()-1, SpecificPath);
      int RobotTrajIndex = EndEffectorSelector(ExeTimeVec, EndDistVec, DistTol);
      std::printf("Planning successfully finds a feasible solution! \nRobot Limb Index: %d\n", NonlinearOptimizerInfo::RobotLinkInfo[RobotTrajVec[RobotTrajIndex].SwingLimbIndex].LinkIndex);
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
