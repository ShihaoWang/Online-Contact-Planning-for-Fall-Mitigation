// This function is used to extensively simulate the result from the four failure metric
#include "RobotInfo.h"
#include "CommonHeader.h"
#include <ode/ode.h>
#include "Control/PathController.h"
#include "Control/JointTrackingController.h"
#include "NonlinearOptimizerInfo.h"

bool SimulationTest(WorldSimulation & Sim, std::vector<LinkInfo> & RobotLinkInfo, std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject, const string & SpecificPath, const int & FileIndex, const Vector3 & ImpulseForceMax)
{
  /* Simulation parameters */
  double  TimeStep          = 0.025;
  int DOF = Sim.world->robots[0]->q.size();
  double  COMFailureDist  = 0.35;
  double  InitDuration    = 2.0;
  double  PushPeriod      = 5.0;                                    // Every 10s a push will be given to the robot body.
  double  PushTriTime     = PushPeriod;                             // Initial disturbance is given.
  double  PushDuration    = 0.5;                                    // Push lasts for 0.5s.
  double  PushDurationMeasure = 0.0;                                // To measure how long push has been imposed to the robot body.
  int     PushGeneFlag    = 0;                                      // For the generation of push magnitude.
  int     PushControlFlag = 0;                                      // Robot will switch to push recovery controller when PushControlFlag = 1;
  double  DetectionWait = 0.1;                                      // After the push controller finishes, we would like to pause for sometime before failure detection!
  double  DetectionWaitMeasure = 1.0;
  double  SimTotalTime    = 5.0;                                    // Simulation lasts for 10s.

  std::vector<string> EdgeFileNames = EdgeFileNamesGene(SpecificPath, FileIndex);
  // Three types of trajectories should be saved for visualization purpose.
  string FailureStateTrajStr = SpecificPath + std::to_string(FileIndex) + "/FailureStateTraj.path";
  const char *FailureStateTrajStr_Name = FailureStateTrajStr.c_str();
  string CtrlStateTrajStr = SpecificPath + std::to_string(FileIndex) + "/CtrlStateTraj.path";
  const char *CtrlStateTrajStr_Name = CtrlStateTrajStr.c_str();
  string PlanStateTrajFileStr = SpecificPath + std::to_string(FileIndex) + "/PlanStateTraj.path";
  const char *PlanStateTrajStr_Name = PlanStateTrajFileStr.c_str();

  /* Override the default controller with a PolynomialPathController */
  auto NewControllerPtr = std::make_shared<PolynomialPathController>(*Sim.world->robots[0]);
  Sim.SetController(0, NewControllerPtr);
  NewControllerPtr->SetConstant(Sim.world->robots[0]->q);

  std::vector<double> qDes = Sim.world->robots[0]->q;               // This is commanded robot configuration to the controller.
  Robot SimRobot = *Sim.world->robots[0];
  ControlReferenceInfo ControlReference;                            // Used for control reference generation.
  Vector3 ImpulseForce;

  LinearPath FailureStateTraj, CtrlStateTraj, PlanStateTraj;

  // The first part is to simualte the robot for a certain amount of time.
  while(Sim.time <= InitDuration)
  {
    FailureStateTraj.Append(Sim.time, Sim.world->robots[0]->q);
    CtrlStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
    PlanStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
    // std::cout<<Sim.world->robots[0]->q[2]<<endl;
    std::printf("Initial Simulation Time: %f\n", Sim.time);

    if((Sim.world->robots[0]->q[2])<0.35)
    {
      return false;
    }

    StateTrajAppender(FailureStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
    StateTrajAppender(CtrlStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
    StateTrajAppender(PlanStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);

    Sim.Advance(TimeStep);
    Sim.UpdateModel();
  }
  SimTotalTime           += Sim.time;
  std::printf("Initial Simulation Done!\n");

  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(SimRobot, COMPos, COMVel);
  double COMDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(COMPos);

  double InitTime = Sim.time;
  double CurTime = Sim.time;

  FailureStateInfo FailureStateObj;

  while(Sim.time <= SimTotalTime)
  {
    /*
      This main loop will be terminated if the simulation time has passed or robot falls to the environment.
    */
    SimRobot = *Sim.world->robots[0];
    if(PushTriTime >= PushPeriod)
    {
      switch (PushGeneFlag)
      {
        case 0:
        {
          // Flat:
          // 1Contact
          double FxBnd = ImpulseForceMax.x, FyBnd = ImpulseForceMax.y, FzBnd = ImpulseForceMax.z;
          ImpulseForce = ImpulForceGene(FxBnd, FyBnd, FzBnd);
          PushGeneFlag = 1;
        }
        break;
        default:
        break;
      }

      if(PushDurationMeasure <= PushDuration)
      {
        // Push should last in this duration.
        PushDurationMeasure+=TimeStep;
        double ImpulseScale = 1.0 * PushDurationMeasure/PushDuration;
        dBodyAddForceAtPos(Sim.odesim.robot(0)->body(19), ImpulseScale * ImpulseForce.x, ImpulseScale * ImpulseForce.y, ImpulseScale * ImpulseForce.z, 0.0, 0.0, 0.0);     // Body 2
        PushInfoFileAppender(Sim.time, ImpulseForce.x, ImpulseForce.y, ImpulseForce.z, SpecificPath, FileIndex);
      }
      else
      {
        PushTriTime = 0.0;
        PushDurationMeasure = 0.0;
        PushGeneFlag = 0;
      }

    }
    /* Robot's COMPos and COMVel */
    CentroidalState(SimRobot, COMPos, COMVel);
    std::vector<Vector3> ActContactPos = ContactPositionFinder(SimRobot, RobotLinkInfo, RobotContactInfo);
    std::vector<PIPInfo> PIPTotal = PIPGenerator(ActContactPos, COMPos, COMVel);
    int CPPIPIndex;
    double RefFailureMetric = CapturePointGenerator(PIPTotal, CPPIPIndex);
    double EndEffectorDist = PresumeContactMinDis(SimRobot, RobotContactInfo);

    std::cout<<"EndEffectorDist: "<<EndEffectorDist<<endl;

    if(SimRobot.q[2]<0.35)
    {
      return false;
    }
    std::printf("Simulation Time: %f and Failure Metric: %f\n", Sim.time, RefFailureMetric);
    ContactPolytopeWriter(PIPTotal, EdgeFileNames);
    switch (PushControlFlag)
    {
      case 1:
      {
        CurTime = Sim.time;
        qDes = ControlReference.ConfigReference(InitTime, CurTime);
        if(((CurTime - InitTime)>ControlReference.PlanStateTraj.EndTime())&&(EndEffectorDist<=0.005))
        {
          // Turn off PushControlFlag
          RobotContactInfo = ControlReference.GoalContactInfo;
          PushControlFlag = 0;
          DetectionWaitMeasure = 0.0;
          InitTime = Sim.time;
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
            if(DetectionWaitMeasure>DetectionWait)
            {
              InitTime = Sim.time;
              switch (FailureStateObj.FailureInitFlag)
              {
                case false:
                FailureStateObj.FailureStateUpdate(InitTime, SimRobot.q, SimRobot.dq);
                break;
                default:
                break;
              }
              // Push recovery controller reference should be computed here.
              // Here a configuration generator should be produced such that at each time, a configuration reference is avaiable for controller to track.
              double PlanTime;
              // std::cout<<SimRobot.q[2]<<endl;
              ControlReference = ControlReferenceGeneration(SimRobot, PIPTotal[CPPIPIndex], RefFailureMetric, RobotContactInfo, RMObject, TimeStep, PlanTime);
              if(ControlReference.ControlReferenceFlag == true)
              {
                PlanTimeRecorder(PlanTime, SpecificPath, FileIndex);
                PushControlFlag = 1;
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
    PlanStateTraj.Append(Sim.time,    Config(qDes));

    StateTrajAppender(CtrlStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
    StateTrajAppender(PlanStateTrajStr_Name, Sim.time, qDes);

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
    PushTriTime+=TimeStep;
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
  switch (FailureStateObj.FailureInitFlag)
  {
    case true:
    {
      Sim.time = FailureStateObj.FailureTime;

      Sim.world->robots[0]->UpdateConfig(FailureStateObj.FailureConfig);
      Sim.world->robots[0]->dq = FailureStateObj.FailureVelocity;

      Sim.controlSimulators[0].oderobot->SetConfig(FailureStateObj.FailureConfig);
      Sim.controlSimulators[0].oderobot->SetVelocities(FailureStateObj.FailureVelocity);

      NewControllerPtr->SetConstant(FailureStateObj.FailureConfig);

      while(Sim.time <= CtrlStateTraj.EndTime())
      {
        FailureStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
        StateTrajAppender(FailureStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
        Sim.Advance(TimeStep);
        Sim.UpdateModel();
      }
    }
    break;
    default:
    break;
  }

  // Write these three trajectories into files.
  ofstream FailureStateTrajFile;
  FailureStateTrajFile.open (FailureStateTrajStr_Name);
  FailureStateTraj.Save(FailureStateTrajFile);
  FailureStateTrajFile.close();

  return FailureStateObj.FailureInitFlag;
}
