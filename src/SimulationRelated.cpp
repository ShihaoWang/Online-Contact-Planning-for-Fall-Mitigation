#include "RobotInfo.h"
#include "CommonHeader.h"
#include <ode/ode.h>
#include "Control/PathController.h"
#include "Control/JointTrackingController.h"
#include "NonlinearOptimizerInfo.h"

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

std::vector<double> OnlineConfigReference(WorldSimulation & Sim, double & InitTime, ControlReferenceInfo & ControlReference, AnyCollisionGeometry3D & TerrColGeom, SelfLinkGeoInfo & SelfLinkGeoObj, double & DetectionWaitMeasure, bool & InMPCFlag, std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject)
{
  double ExeTimeTol = 0.2;
  double TouchTol  = 0.025;             //  2.5 cm as a Touch Down tolerance.
  Robot SimRobot = *Sim.world->robots[0];
  std::vector<double> qDes;
  double CurTime = Sim.time;
  double InnerTime = CurTime - InitTime;

  if(ControlReference.TouchDownConfigFlag) return ControlReference.TouchDownConfig;
  qDes = ControlReference.ConfigReference(InitTime, CurTime);

  std::vector<double> SwingContactDistVec;
  Vector3 SwingLimbContactPos;
  for (Vector3 & LocalContact:NonlinearOptimizerInfo::RobotLinkInfo[ControlReference.SwingLimbIndex].LocalContacts) {
    SimRobot.GetWorldPosition(LocalContact, NonlinearOptimizerInfo::RobotLinkInfo[ControlReference.SwingLimbIndex].LinkIndex, SwingLimbContactPos);
    double CurrentDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(SwingLimbContactPos);
    SwingContactDistVec.push_back(CurrentDist);
  }
  double SwingContactDist = *min_element(SwingContactDistVec.begin(), SwingContactDistVec.end());
  bool OptFlag;
  if(ControlReference.FinalTime>ExeTimeTol){
    if(InnerTime>=ControlReference.SwitchTime){
      if(SwingContactDist<=TouchTol){
        // Run a configuration optimization to make sure that a facet contact can be estabilished at swing limb
        SelfLinkGeoObj.LinkBBsUpdate(SimRobot);
        std::vector<double> qDesTouch = TouchDownConfigOptFn(SimRobot, ControlReference.SwingLimbIndex, SelfLinkGeoObj, RMObject, OptFlag);
        if(OptFlag) qDes = qDesTouch;
        ControlReference.TouchDownConfigFlag = true;
        ControlReference.TouchDownConfig = qDes;
        InMPCFlag = false;
        DetectionWaitMeasure = 0.0;
        RobotContactInfo = ControlReference.GoalContactInfo;
      }
    }
  }else{
    if(SwingContactDist<=TouchTol){
      // Run a configuration optimization to make sure that a facet contact can be estabilished at swing limb
      SelfLinkGeoObj.LinkBBsUpdate(SimRobot);
      std::vector<double> qDesTouch = TouchDownConfigOptFn(SimRobot, ControlReference.SwingLimbIndex, SelfLinkGeoObj, RMObject, OptFlag);
      if(OptFlag) qDes = qDesTouch;
      ControlReference.TouchDownConfigFlag = true;
      ControlReference.TouchDownConfig = qDes;
      InMPCFlag = false;
      DetectionWaitMeasure = 0.0;
      RobotContactInfo = ControlReference.GoalContactInfo;
    }
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

  StateTrajAppender(PlanStateTrajStr_Name, Sim.time, qDes);
  PlanStateTraj.Append(Sim.time,    Config(qDes));

  if(!FailureStateObj.FailureInitFlag)
  {
    FailureStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
    StateTrajAppender(FailureStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
  }
  return;
}
