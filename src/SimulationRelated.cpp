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

std::vector<double> OnlineConfigReference(WorldSimulation & Sim, double & InitTime, ControlReferenceInfo & ControlReference, AnyCollisionGeometry3D & TerrColGeom, SelfLinkGeoInfo & SelfLinkGeoObj, const double & DisTol, double & DetectionWaitMeasure, bool & PushRecovFlag, std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject)
{
  Robot SimRobot = *Sim.world->robots[0];
  std::vector<double> qDes;
  double CurTime = Sim.time;
  double InnerTime = CurTime - InitTime;
  if(InnerTime<ControlReference.SwitchTime)
  {
    qDes = ControlReference.ConfigReference(InitTime, CurTime);
  }
  else
  {
    if(InnerTime<=ControlReference.FinalTime)
    {
      // Here an individual optimization needs to be conducted where robot's actual configuration should be considered.
      std::vector<int> ActiveJoint = RMObject.EndEffectorLink2Pivotal[ControlReference.SwingLimbIndex];
      SelfLinkGeoObj.LinkBBsUpdate(SimRobot);
      qDes = LastControlReference(SimRobot, InitTime, CurTime, ControlReference, SelfLinkGeoObj, RMObject);
    }
  }
  double SwingLimbSignedDist = SimRobot.geometry[NonlinearOptimizerInfo::RobotLinkInfo[ControlReference.SwingLimbIndex].LinkIndex]->Distance(TerrColGeom);
  if(((CurTime - InitTime)>ControlReference.PlanStateTraj.EndTime())&&(SwingLimbSignedDist<=DisTol))
  {
    DetectionWaitMeasure = 0.0;
    InitTime = Sim.time;
    RobotContactInfo = ControlReference.GoalContactInfo;
    PushRecovFlag = false;
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
