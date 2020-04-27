#include "RobotInfo.h"
#include "CommonHeader.h"
#include <ode/ode.h>
#include "Control/PathController.h"
#include "Control/JointTrackingController.h"
#include "NonlinearOptimizerInfo.h"

static double EndEffectorProjTol = 0.995;

void InitialSimulation(WorldSimulation & Sim, LinearPath & FailureStateTraj, LinearPath & CtrlStateTraj, LinearPath & PlanStateTraj, const double & InitDuration, const double & TimeStep, const string & SpecificPath)
{
  string FailureStateTrajStr =  SpecificPath + "FailureStateTraj.path";
  const char *FailureStateTrajStr_Name = FailureStateTrajStr.c_str();
  string CtrlStateTrajStr =     SpecificPath + "CtrlStateTraj.path";
  const char *CtrlStateTrajStr_Name = CtrlStateTrajStr.c_str();
  string PlanStateTrajFileStr = SpecificPath + "PlanStateTraj.path";
  const char *PlanStateTrajStr_Name = PlanStateTrajFileStr.c_str();

  while(Sim.time <= InitDuration)
  {
    FailureStateTraj.Append(Sim.time, Sim.world->robots[0]->q);
    CtrlStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
    PlanStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
    std::printf("Initial Simulation Time: %f\n", Sim.time);

    StateTrajAppender(FailureStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
    StateTrajAppender(CtrlStateTrajStr_Name,    Sim.time, Sim.world->robots[0]->q);
    StateTrajAppender(PlanStateTrajStr_Name,    Sim.time, Sim.world->robots[0]->q);

    Sim.Advance(TimeStep);
    Sim.UpdateModel();
  }
  return;
}

void PushImposer(WorldSimulation & Sim, const Vector3 & ImpulseForceMax, const double & InitTime, const double & PushDuration, const int & FailureFlag, const string & SpecificPath)
{
  double CurTime = Sim.time - InitTime;
  if((CurTime<PushDuration)&&(FailureFlag == 0))
  {
    // Push until fall has been detected.
    int LinkIndex = 19;
    double ImpulseScale = 1.0 * CurTime/PushDuration;
    Vector3 ImpulseForce = ImpulseScale * ImpulseForceMax;
    dBodyAddForceAtRelPos(Sim.odesim.robot(0)->body(LinkIndex), ImpulseForce.x, ImpulseForce.y, ImpulseForce.z, 0.0, 0.0, 0.0);
    PushInfoFileAppender(Sim.time, ImpulseForce.x, ImpulseForce.y, ImpulseForce.z, SpecificPath);
  }
}

std::vector<double> RawOnlineConfigReference(WorldSimulation & Sim, double & InitTime, ControlReferenceInfo & ControlReference, AnyCollisionGeometry3D & TerrColGeom, SelfLinkGeoInfo & SelfLinkGeoObj, double & DetectionWaitMeasure, bool & InMPCFlag, std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject)
{
  Robot SimRobot = *Sim.world->robots[0];
  std::vector<double> qDes;
  double CurTime = Sim.time;
  double InnerTime = CurTime - InitTime;

  std::vector<double> SwingContactDistVec;
  Vector3 SwingLimbContactPos;
  Vector3 SwingLimbAvgPos;
  for (Vector3 & LocalContact:NonlinearOptimizerInfo::RobotLinkInfo[ControlReference.SwingLimbIndex].LocalContacts) {
    SimRobot.GetWorldPosition(LocalContact, NonlinearOptimizerInfo::RobotLinkInfo[ControlReference.SwingLimbIndex].LinkIndex, SwingLimbContactPos);
    double CurrentDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(SwingLimbContactPos);
    SwingContactDistVec.push_back(CurrentDist);
  }
  double SwingContactDist = *min_element(SwingContactDistVec.begin(), SwingContactDistVec.end());
  SimRobot.GetWorldPosition(NonlinearOptimizerInfo::RobotLinkInfo[ControlReference.SwingLimbIndex].AvgLocalContact, NonlinearOptimizerInfo::RobotLinkInfo[ControlReference.SwingLimbIndex].LinkIndex, SwingLimbAvgPos);

  if(ControlReference.TouchDownConfigFlag) return ControlReference.TouchDownConfig;
  if(ControlReference.PlanStateTraj.times.size()==0)
  {
    qDes = SimRobot.q;
    ControlReference.TouchDownConfigFlag = true;
    ControlReference.TouchDownConfig = qDes;
    return qDes;
  }
  qDes = ControlReference.ConfigReference(InitTime, CurTime);

  Vector EndEffectorPos;
  double EndEffectorProj;
  ControlReference.EndEffectorTraj.Eval(InnerTime, EndEffectorPos);
  bool InterpolationFlag;
  Vector3 EndEffectorPos3D(EndEffectorPos);
  if(!ControlReference.Type){
    std::vector<double> qDesref = EuclideanInterOptFn(SimRobot, ControlReference.SwingLimbIndex, EndEffectorPos3D, SelfLinkGeoObj, RMObject, InterpolationFlag, EndEffectorProj);
    if(InterpolationFlag) qDes = qDesref;
  }

  // For TouchDown configuration
  if(InnerTime>=ControlReference.FinalTime){
    ControlReference.TouchDownConfigFlag = true;
    ControlReference.TouchDownConfig = qDes;
    InMPCFlag = false;
    DetectionWaitMeasure = 0.0;
    RobotContactInfo = ControlReference.GoalContactInfo;
  }
  return qDes;
}

std::vector<double> OnlineConfigReference(WorldSimulation & Sim, double & InitTime, ControlReferenceInfo & ControlReference, AnyCollisionGeometry3D & TerrColGeom, SelfLinkGeoInfo & SelfLinkGeoObj, double & DetectionWaitMeasure, bool & InMPCFlag, std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject, int & PlanningSteps, const string & SpecificPath, double & MPCCount)
{
  double AddTouchTol  = 0.1;                                //  10 cm as an Addition Contact Landing Tolerance.
  double ModiTouchTol  = 0.1;                               //  10 cm as a Contact Modification Landing Tolerance.
  double TouchDownTol  = 0.01;                             //  2.5 cm as a Touch Down Tolerance.
  double RunningTimeTol = 0.15;                             //  After this time of period, we all consider to be landing phase!
  Robot SimRobot = *Sim.world->robots[0];
  std::vector<double> qDes;
  double CurTime = Sim.time;
  double InnerTime = CurTime - InitTime;

  std::vector<double> SwingContactDistVec;
  Vector3 SwingLimbContactPos;
  Vector3 SwingLimbAvgPos;
  for (Vector3 & LocalContact:NonlinearOptimizerInfo::RobotLinkInfo[ControlReference.SwingLimbIndex].LocalContacts) {
    SimRobot.GetWorldPosition(LocalContact, NonlinearOptimizerInfo::RobotLinkInfo[ControlReference.SwingLimbIndex].LinkIndex, SwingLimbContactPos);
    double CurrentDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(SwingLimbContactPos);
    SwingContactDistVec.push_back(CurrentDist);
  }
  double SwingContactDist = *min_element(SwingContactDistVec.begin(), SwingContactDistVec.end());
  SimRobot.GetWorldPosition(NonlinearOptimizerInfo::RobotLinkInfo[ControlReference.SwingLimbIndex].AvgLocalContact, NonlinearOptimizerInfo::RobotLinkInfo[ControlReference.SwingLimbIndex].LinkIndex, SwingLimbAvgPos);
  bool OptFlag;
  if(ControlReference.TouchDownConfigFlag) return ControlReference.TouchDownConfig;
  qDes = ControlReference.ConfigReference(InitTime, CurTime);
  if(ControlReference.TouchDownPlanningFlag) MPCCount = 0.0;

  Vector EndEffectorPos;
  double EndEffectorProj;
  ControlReference.EndEffectorTraj.Eval(InnerTime, EndEffectorPos);
  bool InterpolationFlag;
  Vector3 EndEffectorPos3D(EndEffectorPos);
  if(!ControlReference.Type){
    std::vector<double> qDesref = EuclideanInterOptFn(SimRobot, ControlReference.SwingLimbIndex, EndEffectorPos3D, SelfLinkGeoObj, RMObject, InterpolationFlag, EndEffectorProj);
    if(InterpolationFlag){
      qDes = qDesref;
      if(EndEffectorProj<EndEffectorProjTol){
        std::vector<double> qDesEndRef = EndEffectorOriOptFn(SimRobot, SimRobot.q, ControlReference.SwingLimbIndex, ControlReference.GoalContactGrad, RMObject, OptFlag, EndEffectorProjTol);
        if(OptFlag) qDes = qDesEndRef;
      }
    }else{  // End Effector Orientation has to be optimized
      std::vector<double> qDesEndRef = EndEffectorOriOptFn(SimRobot, SimRobot.q, ControlReference.SwingLimbIndex, ControlReference.GoalContactGrad, RMObject, OptFlag, EndEffectorProjTol);
      if(OptFlag) qDes = qDesEndRef;
    }
  }

  SimRobot = *Sim.world->robots[0];
  bool TouchDownPhaseFlag = false;
  Vector3 COMPos, COMVel;
  CentroidalState(SimRobot, COMPos, COMVel);
  double PlanTime;
  SelfLinkGeoObj.LinkBBsUpdate(SimRobot);

  // For TouchDown configuration
  if(ControlReference.Type){    // Contact Addition
    if((SwingContactDist<=AddTouchTol)&&(!ControlReference.TouchDownPlanningFlag)){
      ControlReferenceInfo TouchDownControlReference = TouchDownControlReferenceGeneration(SimRobot, COMPos, COMVel, RobotContactInfo, ControlReference.SwingLimbIndex, 1, RMObject, SelfLinkGeoObj, PlanTime, SpecificPath, PlanningSteps, ControlReference.ContactStatusOptionIndex, CurTime);
      if(TouchDownControlReference.ControlReferenceFlag){
        PlanTimeRecorder(PlanTime, SpecificPath);
        ControlReference = TouchDownControlReference;
        ControlReference.TouchDownPlanningFlag = true;
        MPCCount = 0.0;
      }
    }
    if(SwingContactDist<=TouchDownTol){
      TouchDownPhaseFlag = true;
      std::vector<double> qDesTouch = TouchDownConfigOptFn(SimRobot, ControlReference.SwingLimbIndex, SwingLimbAvgPos, SwingContactDist, SelfLinkGeoObj, RMObject, OptFlag);
      if(OptFlag) qDes = qDesTouch;
    }
  }else{                        // Contact Modification
    if(ControlReference.RunningTime>=RunningTimeTol){
      if((SwingContactDist<=ModiTouchTol)&&(!ControlReference.TouchDownPlanningFlag)){
        ControlReferenceInfo TouchDownControlReference = TouchDownControlReferenceGeneration(SimRobot, COMPos, COMVel, RobotContactInfo, ControlReference.SwingLimbIndex, 0, RMObject, SelfLinkGeoObj, PlanTime, SpecificPath, PlanningSteps, ControlReference.ContactStatusOptionIndex, CurTime);
        if(TouchDownControlReference.ControlReferenceFlag){
          PlanTimeRecorder(PlanTime, SpecificPath);
          ControlReference = TouchDownControlReference;
          ControlReference.TouchDownPlanningFlag = true;
          MPCCount = 0.0;
        }
      }
      if(SwingContactDist<=TouchDownTol){
        TouchDownPhaseFlag = true;
        std::vector<double> qDesTouch = TouchDownConfigOptFn(SimRobot, ControlReference.SwingLimbIndex, SwingLimbAvgPos, SwingContactDist, SelfLinkGeoObj, RMObject, OptFlag);
        if(OptFlag) qDes = qDesTouch;
      }
    }
  }

  if(TouchDownPhaseFlag){
    ControlReference.TouchDownPlanningFlag = false;
    ControlReference.TouchDownConfigFlag = true;
    ControlReference.TouchDownConfig = qDes;
    InMPCFlag = false;
    DetectionWaitMeasure = 0.0;
    RobotContactInfo = ControlReference.GoalContactInfo;
  }
  return qDes;
}

void StateLogger(WorldSimulation & Sim, FailureStateInfo & FailureStateObj, LinearPath & CtrlStateTraj, LinearPath & PlanStateTraj, LinearPath & FailureStateTraj, std::vector<double> & qDes, const string & SpecificPath)
{
  string FailureStateTrajStr =  SpecificPath + "FailureStateTraj.path";
  const char *FailureStateTrajStr_Name = FailureStateTrajStr.c_str();
  string CtrlStateTrajStr =     SpecificPath + "CtrlStateTraj.path";
  const char *CtrlStateTrajStr_Name = CtrlStateTrajStr.c_str();
  string PlanStateTrajFileStr = SpecificPath + "PlanStateTraj.path";
  const char *PlanStateTrajStr_Name = PlanStateTrajFileStr.c_str();

  CtrlStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
  StateTrajAppender(CtrlStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);

  if(qDes.size()==0) qDes = PlanStateTraj.milestones[PlanStateTraj.times.size()-1];

  StateTrajAppender(PlanStateTrajStr_Name, Sim.time, qDes);
  PlanStateTraj.Append(Sim.time,    Config(qDes));

  if(!FailureStateObj.FailureInitFlag)
  {
    FailureStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
    StateTrajAppender(FailureStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
  }
  return;
}
