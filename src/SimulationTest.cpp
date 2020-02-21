// This function is used to extensively simulate the result from the four failure metric
#include "RobotInfo.h"
#include "CommonHeader.h"
#include <ode/ode.h>
#include "Control/PathController.h"
#include "Control/JointTrackingController.h"
#include "NonlinearOptimizerInfo.h"

static double DisTol = 0.01;      // 1cm

void SimulationTest(WorldSimulation & Sim, std::vector<LinkInfo> & RobotLinkInfo, std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject, AnyCollisionGeometry3D & TerrColGeom, SelfLinkGeoInfo & SelfLinkGeoObj, const string & SpecificPath, int & PushRecovSuccFlag, int & ActualFailureFlag)
{
  /* Simulation parameters */
  double  TimeStep        = 0.025;
  int DOF = Sim.world->robots[0]->q.size();
  double  InitDuration    = 2.0;
  double  PushDuration    = 0.5;                                      // Push lasts for 0.5s.
  double  PushDurationMeasure = 0.0;                                  // To measure how long push has been imposed to the robot body.
  double  DetectionWait   = 0.5;                                      // After the push controller finishes, we would like to pause for sometime before failure detection!
  double  DetectionWaitMeasure = DetectionWait;
  double  SimTotalTime    = 5.0;                                      // Simulation lasts for 5s.
  int     PushRecovFlag   = 0;                                        // Robot will switch to push recovery controller when PushRecovFlag = 1;
  int     PlanRecorFlag   = 1;                                        // When its value is 1, this means that currently PlanRecords current

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
  SimTotalTime           += Sim.time;
  std::printf("Initial Simulation Done!\n");

  std::vector<double> qDes = PlanStateTraj.milestones[PlanStateTraj.milestones.size()-1];               // This is commanded robot configuration to the controller.
  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  double InitTime = Sim.time;
  double CurTime = Sim.time;

  ControlReferenceInfo ControlReference;                            // Used for control reference generation.
  FailureStateInfo FailureStateObj;

  double ForceMag = 2000.0;     // Flat Contact 1

  Vector3 ImpulseDirection = ImpulseDirectionGene(*Sim.world->robots[0], NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfo);
  Vector3 ImpulseForceMax = ForceMag * ImpulseDirection;

  Robot SimRobot;
  // This loop is used for push recovery experimentation.
  while(Sim.time <= SimTotalTime)
  {
    SimRobot = *Sim.world->robots[0];
    if(PushDurationMeasure<PushDuration)
    {
      int LinkIndex = 19;
      PushDurationMeasure+=TimeStep;
      double ImpulseScale = 1.0 * PushDurationMeasure/PushDuration;
      Vector3 ImpulseForce = ImpulseScale * ImpulseForceMax;
      // Frame3D LinkLocalFrame = SimRobot.links[LinkIndex].T_World;
      // Vector3 ImpulseForceLocal = ImpulseForce;
      // LinkLocalFrame.mulPointInverse(ImpulseForce + LinkLocalFrame.t, ImpulseForceLocal);
      // dBodyAddRelForceAtRelPos(Sim.odesim.robot(0)->body(LinkIndex), ImpulseForceLocal.x, ImpulseForceLocal.y, ImpulseForceLocal.z, 0.0, 0.0, 0.0);
      dBodyAddForceAtRelPos(Sim.odesim.robot(0)->body(LinkIndex), ImpulseForce.x, ImpulseForce.y, ImpulseForce.z, 0.0, 0.0, 0.0);
      PushInfoFileAppender(Sim.time, ImpulseForce.x, ImpulseForce.y, ImpulseForce.z, SpecificPath);
    }

    /* Robot's COMPos and COMVel */
    CentroidalState(SimRobot, COMPos, COMVel);
    std::vector<Vector3> ActContactPos = ContactPositionFinder(SimRobot, RobotLinkInfo, RobotContactInfo);
    std::vector<PIPInfo> PIPTotal = PIPGenerator(ActContactPos, COMPos, COMVel);
    ContactPolytopeWriter(PIPTotal, EdgeFileNames);

    int CPPIPIndex;
    double RefFailureMetric = CapturePointGenerator(PIPTotal, CPPIPIndex);
    double EndEffectorDist = PresumeContactMinDis(SimRobot, RobotContactInfo);

    std::printf("Simulation Time: %f, EndEffectorDist: %f, and Failure Metric: %f\n", Sim.time, EndEffectorDist, RefFailureMetric);
    switch (PushRecovFlag)
    {
      case 1:
      {
        CurTime = Sim.time;
        qDes = ControlReference.ConfigReference(InitTime, CurTime);
        if(((CurTime - InitTime)>ControlReference.PlanStateTraj.EndTime())&&(EndEffectorDist<=DisTol))
        {
          DetectionWaitMeasure = 0.0;
          InitTime = Sim.time;
          RobotContactInfo = ControlReference.GoalContactInfo;
          PushRecovFlag = 0;
          PlanRecorFlag = 1;
        }
      }
      break;
      default:
      {
        switch (CPPIPIndex)
        {
          case -1:
          break;
          default:
          {
            if(DetectionWaitMeasure>=DetectionWait)
            {
              InitTime = Sim.time;
              if(!FailureStateObj.FailureInitFlag)
              FailureStateObj.FailureStateUpdate(InitTime, SimRobot.q, SimRobot.dq);

              // Push recovery controller reference should be computed here.
              // Here a configuration generator should be produced such that at each time, a configuration reference is avaiable for controller to track.
              double PlanTime;
              ControlReference = ControlReferenceGeneration(SimRobot, PIPTotal[CPPIPIndex], RefFailureMetric, RobotContactInfo, RMObject, SelfLinkGeoObj, TimeStep, PlanTime);
              if(ControlReference.ControlReferenceFlag == true)
              {
                PlanTimeRecorder(PlanTime, SpecificPath);
                PushRecovFlag = 1;
                PlanRecorFlag = 0;
              }
            }
            else
            {
              DetectionWaitMeasure+=TimeStep;
            }
          }
        }
      }
      break;
    }
    // Send the control command!
    NewControllerPtr->SetConstant(Config(qDes));

    CtrlStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
    StateTrajAppender(CtrlStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);

    switch (PlanRecorFlag)
    {
      case 1:
      {
        StateTrajAppender(PlanStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
        PlanStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
      }
      break;
      default:
      {
        StateTrajAppender(PlanStateTrajStr_Name, Sim.time, qDes);
        PlanStateTraj.Append(Sim.time,    Config(qDes));
      }
      break;
    }

    switch (FailureStateObj.FailureInitFlag)
    {
      case false:
      {
        FailureStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
        StateTrajAppender(FailureStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
      }
      break;
      default:
      break;
    }
    Sim.Advance(TimeStep);
    Sim.UpdateModel();
  }
  if(FailureChecker(SimRobot, TerrColGeom, RMObject, DisTol))
  {
    // This means that Push Recovery fails to stabilize the robot.
    PushRecovSuccFlag = 0;
  }
  else
  {
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
  if(FailureStateObj.FailureInitFlag)
  {
    Sim.time = FailureStateObj.FailureTime;

    Sim.world->robots[0]->UpdateConfig(FailureStateObj.FailureConfig);
    Sim.world->robots[0]->dq = FailureStateObj.FailureVelocity;

    Sim.controlSimulators[0].oderobot->SetConfig(FailureStateObj.FailureConfig);
    Sim.controlSimulators[0].oderobot->SetVelocities(FailureStateObj.FailureVelocity);

    NewControllerPtr->SetConstant(FailureStateObj.FailureConfig);

    while(Sim.time <= CtrlStateTraj.EndTime())
    {
      CentroidalState(*Sim.world->robots[0], COMPos, COMVel);
      FailureStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
      StateTrajAppender(FailureStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
      Sim.Advance(TimeStep);
      Sim.UpdateModel();
    }
    if(FailureChecker(*Sim.world->robots[0], TerrColGeom, RMObject, DisTol))
    {
      // This means that there exists failure.
      ActualFailureFlag = 1;
    }
    else
    {
      // This means that there is no failure at all.
      ActualFailureFlag = 0;
    }
  }
  else
  {
    ActualFailureFlag = 0;
  }
  // Write these three trajectories into files.
  ofstream FailureStateTrajFile;
  FailureStateTrajFile.open (FailureStateTrajStr_Name);
  FailureStateTraj.Save(FailureStateTrajFile);
  FailureStateTrajFile.close();

  return;
}
