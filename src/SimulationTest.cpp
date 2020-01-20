// This function is used to extensively simulate the result from the four failure metric
#include "RobotInfo.h"
#include "CommonHeader.h"
#include <ode/ode.h>
#include "Control/PathController.h"
#include "Control/JointTrackingController.h"
#include "NonlinearOptimizerInfo.h"

static double KETol = 1e-3;

static void ImpulForceGene(double & Fx_t, double & Fy_t, double & Fz_t)
{
  std::random_device rd;
  std::mt19937 gen(rd());

  double ImpFx = 0.0;
  double ImpFy = 10000.0;
  double ImpFz = 0.0;

  std::uniform_real_distribution<> ImpXdis(ImpFx/2.0, ImpFx);
  std::uniform_real_distribution<> ImpYdis(ImpFy/2.0, ImpFy);
  std::uniform_real_distribution<> ImpZdis(ImpFz/2.0, ImpFz);

  double Sign_x_val = ((double) rand() / (RAND_MAX));
  double Sign_y_val = ((double) rand() / (RAND_MAX));
  double Sign_z_val = ((double) rand() / (RAND_MAX));

  double Sign_x, Sign_y, Sign_z;
  if(Sign_x_val<=0.5)
  {
    Sign_x = -1.0;
  }
  else
  {
    Sign_x = 1.0;
  }
  if(Sign_y_val<=0.5)
  {
    Sign_y = -1.0;
  }
  else
  {
    Sign_y = 1.0;
  }
  if(Sign_z_val<=0.5)
  {
    Sign_z = -1.0;
  }
  else
  {
    Sign_z = 1.0;
  }

  Fx_t = Sign_x_val * ImpXdis(gen);
  Fy_t = Sign_y_val * ImpYdis(gen);
  Fz_t = Sign_z_val * ImpZdis(gen);

  Fx_t = Sign_x_val * ImpFx;
  Fy_t = ImpFy;
  Fz_t = Sign_z_val * ImpFz;

  return;
}

void SimulationTest(WorldSimulation & Sim, std::vector<LinkInfo> & RobotLinkInfo, std::vector<ContactStatusInfo> & RobotContactInfo, ReachabilityMap & RMObject, const double & TimeStep, const int & FileIndex)
{
  /* Simulation parameters */
  int DOF = Sim.world->robots[0]->q.size();
  double  PushPeriod      = 30.0;                                   // Every 30s a push will be given to the robot body.
  double  PushTriTime     = PushPeriod;                             // Initial disturbance is given.
  double  PushDuration    = 0.1;                                    // Push lasts for 0.1s.
  double  PushDurationMeasure = 0.0;                                // To measure how long push has been imposed to the robot body.
  int     PushGeneFlag    = 0;                                      // For the generation of push magnitude.
  int     PushControlFlag = 0;                                      // Robot will switch to push recovery controller when PushControlFlag = 1;
  double  SimTotalTime    = 60.0;                                   // Simulation lasts for 10s.
  SimTotalTime           += Sim.time;

  /* Override the default controller with a PolynomialPathController */
  auto NewControllerPtr = std::make_shared<PolynomialPathController>(*Sim.world->robots[0]);
  Sim.SetController(0, NewControllerPtr);
  NewControllerPtr->SetConstant(Sim.world->robots[0]->q);

  double Fx_t, Fy_t, Fz_t;
  std::vector<string> EdgeFileNames = EdgeFileNamesGene(FileIndex);
  string stateTrajFile = "stateTraj" + std::to_string(FileIndex) + ".path";
  const char *stateTrajFile_Name = stateTrajFile.c_str();

  std::vector<double> qDes = Sim.world->robots[0]->q;               // This is commanded robot configuration to the controller.
  Robot SimRobot = *Sim.world->robots[0];
  ControlReferenceInfo ControlReference;                            // Used for control reference generation.

  double InitTime = Sim.time;
  double CurTime = Sim.time;

  while(Sim.time <= SimTotalTime)
  {
    // Main Simulation Loop
    SimRobot = *Sim.world->robots[0];
    if(PushTriTime >= PushPeriod)
    {
      switch (PushGeneFlag)
      {
        case 0:
        {
          ImpulForceGene(Fx_t, Fy_t, Fz_t);
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
        dBodyAddForceAtPos(Sim.odesim.robot(0)->body(19), ImpulseScale * Fx_t, ImpulseScale * Fy_t, ImpulseScale * Fz_t, 0.0, 0.0, 0.0);     // Body 2
        PushInfoFileAppender(Sim.time, Fx_t, Fy_t, Fz_t, FileIndex);
      }
      else
      {
        PushTriTime = 0.0;
        PushDurationMeasure = 0.0;
        PushGeneFlag = 0;
      }
    }

    /* Robot's COMPos and COMVel */
    Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
    CentroidalState(SimRobot, COMPos, COMVel);
    std::vector<Vector3> ActContactPos = ContactPositionFinder(SimRobot, RobotLinkInfo, RobotContactInfo);
    std::vector<Vector3> ProjActContactPos = ProjActContactPosGene(ActContactPos);
    std::vector<PIPInfo> PIPTotal = PIPGenerator(ProjActContactPos, COMPos, COMVel);
    int CPPIPIndex;
    double RefFailureMetric = CapturePointGenerator(PIPTotal, CPPIPIndex);
    ContactPolytopeWriter(PIPTotal, EdgeFileNames);

    switch (PushControlFlag)
    {
      case 1:
      {
        // This inner control loop should be conducted until robot's end effector makes contact with the environment.
        CurTime = Sim.time;
        qDes = ControlReference.ConfigReference(InitTime, CurTime);
        // std::string ConfigPath = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/user/hrp2/";
        // string OptConfigFile = "PushConfig" + std::to_string(CurTime) + ".config";
        // RobotConfigWriter(qDes, ConfigPath, OptConfigFile);
      }
      break;
      default:
      {
        switch (CPPIPIndex)
        {
          case -1:
          std::printf("Simulation Time: %f\n", Sim.time);
          break;
          default:
          {
            InitTime = Sim.time;
            // Push recovery controller reference should be computed here.
            // Here a configuration generator should be produced such that at each time, a configuration reference is avaiable for controller to track.
            ControlReference = ControlReferenceGeneration(SimRobot, PIPTotal[CPPIPIndex], RefFailureMetric, RobotContactInfo, RMObject, TimeStep);
            if(ControlReference.ControlReferenceFlag == true)
            {
              PushControlFlag = 1;
            }
          }
        }
      }
      break;
    }
    // Send the control command!
    NewControllerPtr->SetConstant(Config(qDes));
    StateTrajAppender(stateTrajFile_Name, Sim.time, Sim.world->robots[0]->q);
    Sim.Advance(TimeStep);
    Sim.UpdateModel();
    PushTriTime+=TimeStep;
  }

  return;
}
