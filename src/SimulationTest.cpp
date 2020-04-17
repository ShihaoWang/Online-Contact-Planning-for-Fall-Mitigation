// This function is used to extensively simulate the result from the four failure metric
#include "RobotInfo.h"
#include "CommonHeader.h"
#include <ode/ode.h>
#include "Control/PathController.h"
#include "Control/JointTrackingController.h"
#include "NonlinearOptimizerInfo.h"

static double DisTol = 0.01;      // 1cm

void SimulationTest(WorldSimulation & Sim, std::vector<LinkInfo> & RobotLinkInfo, std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject, AnyCollisionGeometry3D & TerrColGeom, SelfLinkGeoInfo & SelfLinkGeoObj, const string & SpecificPath, const double & ForceMax, const double & PushDuration, const double & DetectionWait, int & PushRecovSuccFlag, int & ActualFailureFlag)
{
  /* Simulation parameters */
  double  TimeStep        = 0.025;
  int DOF = Sim.world->robots[0]->q.size();
  double  InitDuration    = 2.0;
  double  DetectionWaitMeasure = DetectionWait;
  double  SimTotalTime    = 5.0;                                      // Simulation lasts for 5s.
  bool    FailureFlag     = false;
  int     PlanningSteps   = 0;                                        // Total Planning Step Number
  double  MPCCount        = 0.0;
  double  MPCDuration     = 0.1;                                      // Duration for MPC executation until the next planning
  bool    InMPCFlag       = false;
  bool    TouchDownFlag   = true;                                    // Used for final touch down counting

  std::vector<string> EdgeFileNames = EdgeFileNamesGene(SpecificPath);
  // Three types of trajectories should be saved for visualization purpose.
  string FailureStateTrajStr =  SpecificPath + "FailureStateTraj.path";
  const char *FailureStateTrajStr_Name = FailureStateTrajStr.c_str();
  string CtrlStateTrajStr =     SpecificPath + "CtrlStateTraj.path";
  const char *CtrlStateTrajStr_Name = CtrlStateTrajStr.c_str();
  string PlanStateTrajFileStr = SpecificPath + "PlanStateTraj.path";
  const char *PlanStateTrajStr_Name = PlanStateTrajFileStr.c_str();

  /* Override the default controller with a PolynomialPathController */
  auto NewControllerPtr = std::make_shared<PolynomialPathController>(*Sim.world->robots[0]);
  Sim.SetController(0, NewControllerPtr);
  NewControllerPtr->SetConstant(Sim.world->robots[0]->q);

  // Initial Simulation
  LinearPath FailureStateTraj, CtrlStateTraj, PlanStateTraj;
  InitialSimulation(Sim, FailureStateTraj, CtrlStateTraj, PlanStateTraj, InitDuration, TimeStep, SpecificPath);
  SimTotalTime           += Sim.time;
  std::printf("Initial Simulation Done!\n");

  std::vector<double> qDes = PlanStateTraj.milestones[PlanStateTraj.milestones.size()-1];               // This is commanded robot configuration to the controller.
  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  double InitTime = Sim.time;
  double CurTime = Sim.time;

  ControlReferenceInfo ControlReference;                            // Used for control reference generation.
  FailureStateInfo FailureStateObj;

  Vector3 ImpulseDirection = ImpulseDirectionGene(*Sim.world->robots[0], NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfo);
  Vector3 ImpulseForceMax = ForceMax * ImpulseDirection;

  Robot SimRobot;
  int ContactStatusOptionRef = -1, PreviousContactStatusIndex = -1;
  // This loop is used for push recovery experimentation.
  while(Sim.time <= SimTotalTime){
    SimRobot = *Sim.world->robots[0];
    PushImposer(Sim, ImpulseForceMax,  InitTime, PushDuration, FailureFlag, SpecificPath);

    /* Robot's COMPos and COMVel */
    CentroidalState(SimRobot, COMPos, COMVel);
    std::vector<Vector3> ActContactPos = ContactPositionFinder(SimRobot, RobotLinkInfo, RobotContactInfo);    // From ContactInfoActive
    std::vector<PIPInfo> PIPTotal = PIPGenerator(ActContactPos, COMPos, COMVel);
    ContactPolytopeWriter(PIPTotal, EdgeFileNames);

    int CPPIPIndex;
    double RefFailureMetric = CapturePointGenerator(PIPTotal, CPPIPIndex);
    std::printf("Simulation Time: %f, and Failure Metric: %f\n", Sim.time, RefFailureMetric);

    if((InMPCFlag)&&(MPCCount<MPCDuration)){
      qDes = OnlineConfigReference(Sim, InitTime, ControlReference, TerrColGeom, SelfLinkGeoObj, DetectionWaitMeasure, InMPCFlag, RobotContactInfo, RMObject);
      ControlReference.RunningTime+=TimeStep;
      TouchDownFlag = ControlReference.TouchDownConfigFlag;
      MPCCount+=TimeStep;
      printf("MPC count: %f\n", MPCCount);
      if(!InMPCFlag) MPCCount = 0.0;
    }
    else{
      switch (CPPIPIndex){
        case -1:
        {
          if(MPCCount>=MPCDuration){
            bool OptFlag;
            std::vector<double> qDesTouch = TouchDownConfigOptFn(SimRobot, ControlReference.SwingLimbIndex, ControlReference.GoalContactPos, SelfLinkGeoObj, RMObject, OptFlag);
            if(OptFlag) {
              ControlReference.TouchDownConfigFlag = true;
              ControlReference.TouchDownConfig = qDesTouch;
              InMPCFlag = false;
              MPCCount = 0.0;
              DetectionWaitMeasure = 0.0;
              RobotContactInfo = ControlReference.GoalContactInfo;
            }
          }
        }
        break;
        default:{
          FailureFlag = true;
          if(TouchDownFlag){
            if(DetectionWaitMeasure>=DetectionWait){
              InitTime = Sim.time;
              ContactStatusOptionRef = -1;
              if(!FailureStateObj.FailureInitFlag)  FailureStateObj.FailureStateUpdate(InitTime, SimRobot.q, SimRobot.dq);

              double PlanTime;
              SelfLinkGeoObj.LinkBBsUpdate(SimRobot);
              ControlReference = ControlReferenceGeneration(SimRobot, COMPos, COMVel, RefFailureMetric, RobotContactInfo, RMObject, SelfLinkGeoObj, TimeStep, PlanTime, SpecificPath, PlanningSteps, DisTol, ContactStatusOptionRef, PreviousContactStatusIndex);
              if(ControlReference.ControlReferenceFlag){
                PlanTimeRecorder(PlanTime, SpecificPath);
                ContactStatusOptionRef = ControlReference.ContactStatusOptionIndex;
                DetectionWaitMeasure = 0.0;
                PlanningSteps++;
                InMPCFlag = true;
                MPCCount = 0.0;
                TouchDownFlag = false;
              }
            }
            else DetectionWaitMeasure+=TimeStep;
          }else{  // Then this is the MPC planning
            double PlanTime;
            SelfLinkGeoObj.LinkBBsUpdate(SimRobot);
            ControlReferenceInfo ControlReferenceMPC = ControlReferenceGeneration(SimRobot, COMPos, COMVel, RefFailureMetric, RobotContactInfo, RMObject, SelfLinkGeoObj, TimeStep, PlanTime, SpecificPath, PlanningSteps, DisTol, ContactStatusOptionRef, PreviousContactStatusIndex);
            if(ControlReferenceMPC.ControlReferenceFlag){
              double RunningTime = ControlReference.RunningTime;
              ControlReference = ControlReferenceMPC;
              ControlReference.RunningTime+=RunningTime;
              PlanTimeRecorder(PlanTime, SpecificPath);
              ContactStatusOptionRef = ControlReference.ContactStatusOptionIndex;
              DetectionWaitMeasure = 0.0;
              PlanningSteps++;
              InMPCFlag = true;
              MPCCount = 0.0;
              TouchDownFlag = false;
            }else{
              InMPCFlag = true;
              MPCCount = 0.0;
            }
          }
        }
      }
    }
    NewControllerPtr->SetConstant(Config(qDes));
    StateLogger(Sim, FailureStateObj, CtrlStateTraj, PlanStateTraj, FailureStateTraj, qDes, SpecificPath);
    Sim.Advance(TimeStep);
    Sim.UpdateModel();
  }
  if(FailureChecker(SimRobot, TerrColGeom, RMObject, DisTol)){
    PushRecovSuccFlag = 0;
  }
  else{
    // This means that either Push Recovery successfully protects the robot or robot does not even fall.
    PushRecovSuccFlag = 1;
  }

  ofstream CtrlStateTrajFile;
  CtrlStateTrajFile.open (CtrlStateTrajStr_Name);
  CtrlStateTraj.Save(CtrlStateTrajFile);
  CtrlStateTrajFile.close();

  ofstream PlanStateTrajFile;
  PlanStateTrajFile.open (PlanStateTrajStr_Name);
  PlanStateTraj.Save(PlanStateTrajFile);
  PlanStateTrajFile.close();

  // Then we gonna have to simulate the robot's falling trajectory
  if(FailureStateObj.FailureInitFlag){
    Sim.time = FailureStateObj.FailureTime;

    Sim.world->robots[0]->UpdateConfig(FailureStateObj.FailureConfig);
    Sim.world->robots[0]->dq = FailureStateObj.FailureVelocity;

    Sim.controlSimulators[0].oderobot->SetConfig(FailureStateObj.FailureConfig);
    Sim.controlSimulators[0].oderobot->SetVelocities(FailureStateObj.FailureVelocity);

    NewControllerPtr->SetConstant(FailureStateObj.FailureConfig);

    while(Sim.time <= CtrlStateTraj.EndTime()){
      CentroidalState(*Sim.world->robots[0], COMPos, COMVel);
      FailureStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
      StateTrajAppender(FailureStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
      Sim.Advance(TimeStep);
      Sim.UpdateModel();
    }
    if(FailureChecker(*Sim.world->robots[0], TerrColGeom, RMObject, DisTol)){
      // This means that there exists failure.
      ActualFailureFlag = 1;
    }
    else{
      // This means that there is no failure at all.
      ActualFailureFlag = 0;
    }
  }
  else{
    ActualFailureFlag = 0;
  }
  // Write these three trajectories into files.
  ofstream FailureStateTrajFile;
  FailureStateTrajFile.open (FailureStateTrajStr_Name);
  FailureStateTraj.Save(FailureStateTrajFile);
  FailureStateTrajFile.close();

  return;
}
