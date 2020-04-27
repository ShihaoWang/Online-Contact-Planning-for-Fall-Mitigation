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

static std::vector<Vector3> TouchDownOptimalContactSearcher(const Robot & SimRobot, const PIPInfo & PIPObj, const Vector3 & PredictedCOMPos, ReachabilityMap & RMObject, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & FixedRobotContactInfo, const int & SwingLimbIndex, const Vector3 & RefContactPos, DataRecorderInfo & DataRecorderObj)
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
  // Vector3Writer(ActiveReachableContact, "ActiveReachableContact");

  // 1. Self-collision from other end effectors
  std::vector<Vector3> ContactFreeContact = RMObject.ContactFreePointsFinder(RMObject.EndEffectorCollisionRadius[SwingLimbIndex], ActiveReachableContact, ContactFreeInfo);
  switch (ContactFreeContact.size()){
    case 0:return OptimalContact;
    break;
    default:
    break;
  }
  // Vector3Writer(ContactFreeContact, "ContactFreeContact");

  // 2. Supportive
  std::vector<Vector3> SupportContact = SupportContactFinder(COMPos, PIPObj, ContactFreeContact, NonlinearOptimizerInfo::SDFInfo);
  switch (SupportContact.size()){
    case 0:return OptimalContact;
    break;
    default:
    break;
  }
  // Vector3Writer(SupportContact, "SupportContact");
  OptimalContact = SupportContact;
  Vector3 OptimalContactPos = OptimalContact[0];
  Vector3 ContactPosDiff = OptimalContactPos - RefContactPos;
  double PosDiff = ContactPosDiff.normSquared();

  for (int i = 0; i < OptimalContact.size()-1; i++) {
    ContactPosDiff = OptimalContact[i+1] - RefContactPos;
    if(ContactPosDiff.normSquared()<PosDiff)
    {
      OptimalContactPos = OptimalContact[i+1];
      PosDiff = ContactPosDiff.normSquared();
    }
  }

  std::vector<Vector3> ReducedOptimalContact;
  ReducedOptimalContact.push_back(OptimalContactPos);

  // In The End we give all these values
  DataRecorderObj.ActiveReachableContact = ActiveReachableContact;
  DataRecorderObj.ContactFreeContact = ContactFreeContact;
  DataRecorderObj.SupportContact = SupportContact;
  DataRecorderObj.OptimalContact = OptimalContact;
  DataRecorderObj.ReducedOptimalContact = ReducedOptimalContact;
  // Vector3Writer(ReducedOptimalContact, "ReducedOptimalContact");

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

static std::vector<Vector3> LinearTransition(const Vector3 & ContactInit, const Vector3 & ContactGoal, const int & sNumber){
  // A linear path will be constructed between these two input contact points.
  double sDiff = 1.0/(1.0 * sNumber - 1.0);
  double xInit = ContactInit.x;
  double yInit = ContactInit.y;
  double zInit = ContactInit.z;

  double xGoal = ContactGoal.x;
  double yGoal = ContactGoal.y;
  double zGoal = ContactGoal.z;

  double xK = (xGoal - xInit);
  double yK = (yGoal - yInit);
  double zK = (zGoal - zInit);

  std::vector<Vector3> LinearPath;
  for (int i = 0; i < sNumber; i++)
  {
    double sGrid = (1.0 * i) * sDiff;
    double x_i = xInit + xK * sGrid;
    double y_i = yInit + yK * sGrid;
    double z_i = zInit + zK * sGrid;
    Vector3 LinearPoint(x_i, y_i, z_i);
    LinearPath.push_back(LinearPoint);
  }

  return LinearPath;
}

static ControlReferenceInfo TouchDownControlReferenceGenerationInner(const Robot & SimRobot, const PIPInfo & PIPObj, const Vector3 & PredictedCOMPos, ReachabilityMap & RMObject, SelfLinkGeoInfo & SelfLinkGeoObj, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & FixedRobotContactInfo, const int & SwingLimbIndex, const int & Type, DataRecorderInfo & DataRecorderObj)
{
  ControlReferenceInfo ControlReferenceObj;
  Vector3 ContactInit;       // This is the position of the reference contact for robot's active end effector.
  SimRobot.GetWorldPosition(RobotLinkInfo[SwingLimbIndex].AvgLocalContact, RobotLinkInfo[SwingLimbIndex].LinkIndex, ContactInit);
  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(SimRobot, COMPos, COMVel);
  std::vector<Vector3> OptimalContact = TouchDownOptimalContactSearcher(SimRobot, PIPObj, PredictedCOMPos, RMObject, RobotLinkInfo, FixedRobotContactInfo, SwingLimbIndex, ContactInit, DataRecorderObj);
  if(OptimalContact.size()){
    Robot SimRobotInner = SimRobot;
    Vector3 ContactGoal = OptimalContact[0];
    Vector3 ContactGoalGrad = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(ContactGoal);
    InvertedPendulumInfo InvertedPendulumObj(PIPObj.theta, PIPObj.thetadot, COMPos, COMVel);
    Vector3 ContactDiff = ContactGoal - ContactInit;
    double PathLength = sqrt(ContactDiff.normSquared());

    const int sNumber = 5;
    std::vector<Vector3> TransitionPoints = LinearTransition(ContactInit, ContactGoal, sNumber);
    DataRecorderObj.TransitionPoints = TransitionPoints;
    int sIndex = 1;
    double sDiff = 1.0/(1.0 * sNumber - 1.0);
    double sVal = 0.0;
    Config CurrentConfig = SimRobotInner.q;
    CurrentConfig = YPRShifter(CurrentConfig);
    double CurrentTime = 0.0;
    Vector3 CurrentContactPos = ContactInit;

    std::vector<double> TimeTraj;
    std::vector<Config> ConfigTraj;
    std::vector<Vector3> EndEffectorTraj;

    TimeTraj.reserve(sNumber);
    ConfigTraj.reserve(sNumber);
    EndEffectorTraj.reserve(sNumber);

    TimeTraj.push_back(CurrentTime);
    ConfigTraj.push_back(CurrentConfig);
    EndEffectorTraj.push_back(CurrentContactPos);

    bool OptFlag = true;
    while((sIndex<sNumber)&&(OptFlag == true))
    {
      CurrentContactPos = TransitionPoints[sIndex];
      bool LastFlag;
      switch (sIndex)
      {
        case 4:   LastFlag = true;
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
        EndEffectorTraj.push_back(CurrentContactPos);

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
      ControlReferenceObj.TrajectoryUpdate(TimeTraj, ConfigTraj, EndEffectorTraj, ContactGoal, ContactGoalGrad, PathLength, Type);
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

  }else return ControlReferenceObj;

  return ControlReferenceObj;
}

ControlReferenceInfo TouchDownControlReferenceGeneration(Robot & SimRobot, const Vector3 & COMPos, const Vector3 & COMVel, const std::vector<ContactStatusInfo> & RobotContactInfo, const int & SwingLimbIndex, const int & Type, ReachabilityMap & RMObject, SelfLinkGeoInfo & SelfLinkGeoObj, double & PlanTime, const string & SpecificPath, int & PlanningSteps, const int & ContactStatusOptionRef, const double & CurTime)
{
  // This planning function is specific for the planning of touch down reference where the goal is to land the contact at the nearest contact in the environment.
  std::clock_t start_time = std::clock();          // get current time
  int ContactStatusOption = 0;
  int LimbSuccessNo = 0;
  ControlReferenceInfo RobotTraj;
  std::vector<Vector3> ActContactPos = ContactPositionFinder(SimRobot, NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfo);    // From ContactInfoActive

  DataRecorderInfo DataRecorderObj;

  // Here TipOverPIP needs to be computed given RobotContactInfo!
  Vector3 PredictedCOMPos;
  PIPInfo TipOverPIP = TipOverPIPGene(ActContactPos, COMPos, COMVel, PredictedCOMPos);

  RobotTraj = TouchDownControlReferenceGenerationInner(SimRobot, TipOverPIP, PredictedCOMPos, RMObject, SelfLinkGeoObj, NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfo, SwingLimbIndex, Type, DataRecorderObj);
  RobotTraj.SwingLimbIndex = SwingLimbIndex;
  RobotTraj.ContactStatusOptionIndex = ContactStatusOption;
  double duration_time = (std::clock() - start_time)/(double)CLOCKS_PER_SEC;
  std::printf("Touch Down Planning takes: %f ms\n", 1000.0 * duration_time);
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
    PlanningInfoFileAppender(PlanningSteps, 0, SpecificPath, CurTime);
    PlanningSteps++;
  }
  return RobotTraj;
}
