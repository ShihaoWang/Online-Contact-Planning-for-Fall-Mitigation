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
  // double  PushDuration    = 0.2;                                      // Push lasts for 0.5s.
  double  PushDurationMeasure = 0.0;                                  // To measure how long push has been imposed to the robot body.
  // double  DetectionWait   = 0.25;                                      // After the push controller finishes, we would like to pause for sometime before failure detection!
  double  DetectionWaitMeasure = DetectionWait;
  double  SimTotalTime    = 5.0;                                      // Simulation lasts for 5s.
  int     PushRecovFlag   = 0;                                        // Robot will switch to push recovery controller when PushRecovFlag = 1;
  int     FailureFlag     = 0;
  int     PlanningSteps   = 0;                                        // Total Planning Step Number

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

  Vector3 ImpulseDirection = ImpulseDirectionGene(*Sim.world->robots[0], NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfo);
  // Vector3 ImpulseDirection = FlatRandomDirection();
  Vector3 ImpulseForceMax = ForceMax * ImpulseDirection;

  Robot SimRobot;
  // This loop is used for push recovery experimentation.
  while(Sim.time <= SimTotalTime)
  {
    SimRobot = *Sim.world->robots[0];
    if((PushDurationMeasure<PushDuration)&&(FailureFlag == 0))
    {
      // Push until fall has been detected.
      int LinkIndex = 19;
      PushDurationMeasure+=TimeStep;
      double ImpulseScale = 1.0 * PushDurationMeasure/PushDuration;
      Vector3 ImpulseForce = ImpulseScale * ImpulseForceMax;
      dBodyAddForceAtRelPos(Sim.odesim.robot(0)->body(LinkIndex), ImpulseForce.x, ImpulseForce.y, ImpulseForce.z, 0.0, 0.0, 0.0);
      PushInfoFileAppender(Sim.time, ImpulseForce.x, ImpulseForce.y, ImpulseForce.z, SpecificPath);
    }

    /* Robot's COMPos and COMVel */
    CentroidalState(SimRobot, COMPos, COMVel);
    std::vector<Vector3> ActContactPos = ContactPositionFinder(SimRobot, RobotLinkInfo, RobotContactInfo);    // From ContactInfoActive
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
        // However, one effect has been noticed is that the collision impulse.
        CurTime = Sim.time;
        qDes = ControlReference.ConfigReference(InitTime, CurTime);
        if(((CurTime - InitTime)>ControlReference.PlanStateTraj.EndTime())&&(EndEffectorDist<=DisTol))
        {
          DetectionWaitMeasure = 0.0;
          InitTime = Sim.time;
          RobotContactInfo = ControlReference.GoalContactInfo;
          PushRecovFlag = 0;
          qDes = SimRobot.q;
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
            FailureFlag = 1;
            if(DetectionWaitMeasure>=DetectionWait)
            {
              InitTime = Sim.time;
              if(!FailureStateObj.FailureInitFlag)
              FailureStateObj.FailureStateUpdate(InitTime, SimRobot.q, SimRobot.dq);

              double PlanTime;
              SelfLinkGeoObj.LinkBBsUpdate(SimRobot);
              ControlReference = ControlReferenceGeneration(SimRobot, COMPos, COMVel, RefFailureMetric, RobotContactInfo, RMObject, SelfLinkGeoObj, TimeStep, PlanTime, SpecificPath, PlanningSteps, DisTol);
              if(ControlReference.ControlReferenceFlag == true)
              {
                PlanTimeRecorder(PlanTime, SpecificPath);
                PushRecovFlag = 1;
                DetectionWaitMeasure = 0.0;
                PlanningSteps++;
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

    StateTrajAppender(PlanStateTrajStr_Name, Sim.time, qDes);
    PlanStateTraj.Append(Sim.time,    Config(qDes));

    if(!FailureStateObj.FailureInitFlag)
    {
      FailureStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
      StateTrajAppender(FailureStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
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
