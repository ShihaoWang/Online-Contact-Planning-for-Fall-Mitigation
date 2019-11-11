// This function is used to extensively simulate the result from the four failure metric
#include "RobotInfo.h"
#include "CommonHeader.h"
#include <ode/ode.h>
#include "Control/PathController.h"
#include "Control/JointTrackingController.h"

static double KETol = 1e-3;

static Vector3 ImpulForceGene(double & Fx_t, double & Fy_t, double & Fz_t)
{
  std::random_device rd;
  std::mt19937 gen(rd());

  double ImpFx = 7500.0;
  double ImpFy = 7500.0;
  double ImpFz = 1500.0;

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

  std::printf("Sign_x_val: %f\n", Sign_x_val);
  std::printf("Sign_y_val: %f\n", Sign_y_val);
  std::printf("Sign_z_val: %f\n", Sign_z_val);

  Fx_t = Sign_x_val * ImpXdis(gen);
  Fy_t = Sign_y_val * ImpYdis(gen);
  Fz_t = Sign_z_val * ImpZdis(gen);

  Vector3 F_t(Fx_t, Fy_t, Fz_t);
  return F_t;
}

static std::vector<Vector3> ProjActContactPosGene(const std::vector<Vector3> & ActContactPositions)
{
  // Projection to ground
  std::vector<Vector3> ProjActContactPositions;
  ProjActContactPositions.reserve(ActContactPositions.size());
  double LowestHeight = 1000.0;
  for (int j = 0; j < ActContactPositions.size(); j++)
  {
    if(LowestHeight>ActContactPositions[j].z)
    {
      LowestHeight = ActContactPositions[j].z;
    }
  }
  for (int j = 0; j < ActContactPositions.size(); j++)
  {
    Vector3 ProjActContact(ActContactPositions[j].x, ActContactPositions[j].y, LowestHeight);
    ProjActContactPositions.push_back(ProjActContact);
  }
  return ProjActContactPositions;
}

static void SimSmoother(const int & ControllerType, WorldSimulation & Sim, const std::vector<Config> & qdotDesTraj, const int & DOF)
{
  // This function is used to smoothen the weird oscillatory motion at certain robot actuators.
  switch (ControllerType)
  {
    case 1:
    {

    }
    break;
    default:
    {
      // A weird problem with actuators at ankles have been observed. Here we assume that these two motors behave in an ideal way.
      std::vector<double> RealVelocities(DOF);
      for (int i = 0; i < DOF; i++)
      {
        RealVelocities[i] = Sim.world->robots[0]->dq[i];
      }
      // Four actuators are compensated with ideal values.
      RealVelocities[10] = qdotDesTraj[qdotDesTraj.size()-1][10];
      RealVelocities[11] = qdotDesTraj[qdotDesTraj.size()-1][11];
      RealVelocities[16] = qdotDesTraj[qdotDesTraj.size()-1][16];
      RealVelocities[17] = qdotDesTraj[qdotDesTraj.size()-1][17];

      Sim.world->robots[0]->dq = RealVelocities;
      Config RealVelocitiesSet(RealVelocities);
      Sim.controlSimulators[0].oderobot->SetVelocities(RealVelocitiesSet);
    }
    break;
  }
  return;
}

void SimulationTest(WorldSimulation & Sim, std::vector<LinkInfo> & RobotLinkInfo, std::vector<ContactStatusInfo> & RobotContactInfo, SignedDistanceFieldInfo & SDFInfo, SimGUIBackend & Backend, const std::vector<Vector3> & ContactPositionRef, const double & dt, const int & FileIndex)
{
  /* Simulation parameters */
  int     EdgeNumber      = 4;
  double  mu              = 1.0;
  int DOF = Sim.world->robots[0]->q.size();
  double  t_last          = 2.0 * dt;
  double  t_impul         = Sim.time + t_last;                      // The impulse lasts for 2.0 * dt.
  double  t_final         = 10.0;                      // The impulse lasts for 2.0 * dt.
  t_final+=Sim.time;

  double Fx_t, Fy_t, Fz_t;
  Vector3 ImpulseForce = ImpulForceGene(Fx_t, Fy_t, Fz_t);

  /* Override the default controller with a PolynomialPathController */
  auto NewControllerPtr = std::make_shared<PolynomialPathController>(*Sim.world->robots[0]);
  Sim.SetController(0, NewControllerPtr);
  NewControllerPtr->SetConstant(Sim.world->robots[0]->q);

  /* Simulation Trajectory */
  // These two save robot's desired trajectories.
  std::vector<Config> qDesTraj,       qdotDesTraj;
  qDesTraj.push_back(Sim.world->robots[0]->q);
  qdotDesTraj.push_back(Sim.world->robots[0]->dq);

  // These two save robot's actual trajectories.
  std::vector<Config> qActTraj,       qdotActTraj;
  qActTraj.push_back(Sim.world->robots[0]->q);
  qdotActTraj.push_back(Sim.world->robots[0]->dq);

  // Seven objective trajectories
  std::vector<double> PVKRBTraj, PVKCPTraj, OETraj, CPTraj;

  int NumberOfActEndEffectorInit;               // This variable describes the number of active end effectors!
  int NumberOfContactPoints;                    // This variable describes the number of total contact points!
  ContactNumberFinder(RobotContactInfo, NumberOfActEndEffectorInit, NumberOfContactPoints);
  SpecsWriter(*Sim.world->robots[0], t_final, t_last, ImpulseForce, NumberOfActEndEffectorInit, FileIndex);

  std::vector<const char*> EdgeFileNames, StateTrajNames;
  string fEdgeAFile = "EdgeATraj" + std::to_string(FileIndex) + ".txt";                 const char *fEdgeAFile_Name = fEdgeAFile.c_str();
  string fEdgeBFile = "EdgeBTraj" + std::to_string(FileIndex) + ".txt";                 const char *fEdgeBFile_Name = fEdgeBFile.c_str();
  string fEdgeCOMFile = "EdgeCOMTraj" + std::to_string(FileIndex) + ".txt";             const char *fEdgeCOMFile_Name = fEdgeCOMFile.c_str();
  string fEdgexTrajFile = "EdgexTraj" + std::to_string(FileIndex) + ".txt";             const char *fEdgexTrajFile_Name = fEdgexTrajFile.c_str();
  string fEdgeyTrajFile = "EdgeyTraj" + std::to_string(FileIndex) + ".txt";             const char *fEdgeyTrajFile_Name = fEdgeyTrajFile.c_str();
  string fEdgezTrajFile = "EdgezTraj" + std::to_string(FileIndex) + ".txt";             const char *fEdgezTrajFile_Name = fEdgezTrajFile.c_str();

  EdgeFileNames.push_back(fEdgeAFile_Name);
  EdgeFileNames.push_back(fEdgeBFile_Name);
  EdgeFileNames.push_back(fEdgeCOMFile_Name);
  EdgeFileNames.push_back(fEdgexTrajFile_Name);
  EdgeFileNames.push_back(fEdgeyTrajFile_Name);
  EdgeFileNames.push_back(fEdgezTrajFile_Name);

  const string qActTrajFile = "qActTraj" + std::to_string(FileIndex) + ".txt";          const char *qActTrajFile_Name = qActTrajFile.c_str();
  const string qdotActTrajFile = "qdotActTraj" + std::to_string(FileIndex) + ".txt";    const char *qdotActTrajFile_Name = qdotActTrajFile.c_str();
  StateTrajNames.push_back(qActTrajFile_Name);
  StateTrajNames.push_back(qdotActTrajFile_Name);

  string stateTrajFile = "stateTraj" + std::to_string(FileIndex) + ".path";             const char *stateTrajFile_Name = stateTrajFile.c_str();

  /*
    Here we have two types of controller:
        1. Rigid-body controller
        2. Whole-body QP stabilizing controller
  */

  int ControllerType = 2;
  std::vector<double> qDes = Sim.world->robots[0]->q;   // This is commanded robot configuration to the controller.

  // The simulation will be terminated if robot's current kinetic energy declines to a value lower than the tolerance.
  // However, initially this simulation has to last until t_impul time has elapsed.

  Robot SimRobot = *Sim.world->robots[0];
  while(Sim.time <t_impul)
  {
    SimSmoother(ControllerType, Sim, qdotDesTraj, DOF);
    SimRobot = *Sim.world->robots[0];
    // The impulse is given to the robot's torso
    switch (ControllerType)
    {
      case 2:
      {
        dBodyAddForceAtPos(Sim.odesim.robot(0)->body(19), Fx_t, Fy_t, Fz_t, 0.0, 0.0, 0.0);
      }
      break;
      default:
      {
        // For PID controller, disturbances need to be larger since it is more robust than QP controller.
        dBodyAddForceAtPos(Sim.odesim.robot(0)->body(18), Fx_t, Fy_t, Fz_t, 0.0, 0.0, 0.0);
        dBodyAddForceAtPos(Sim.odesim.robot(0)->body(19), Fx_t, Fy_t, Fz_t, 0.0, 0.0, 0.0);
      }
      break;
    }
    qDesTraj.push_back(Sim.world->robots[0]->q);
    qdotDesTraj.push_back(Sim.world->robots[0]->dq);

    /* Robot's COMPos and COMVel */
    Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
    CentroidalState(SimRobot, COMPos, COMVel);

    std::vector<Vector3>  ActContactPositions, ActVelocities;        // A vector of Vector3 points
    std::vector<Matrix>   ActJacobians;       // A vector of Jacobian matrices
    std::vector<int> ActStatus = ActContactNJacobian(SimRobot, RobotLinkInfo, RobotContactInfo, ActContactPositions, ActVelocities, ActJacobians, SDFInfo);

    std::vector<Vector3> ConeShiftedUnits, ConeUnits;
    ConeUnitGenerator(ActContactPositions, SDFInfo, ConeShiftedUnits, ConeUnits, EdgeNumber, mu);

    /*  Controller Input  */
    switch (ControllerType)
    {
      case 1:
      {
        // In this case, the robot's controller holds a constant initial configuration.
        std::printf("Using Controller 1: Rigid-body Controller!\n");

        qActTraj.push_back(SimRobot.q);
        qdotActTraj.push_back(SimRobot.dq);
      }
      break;
      case 2:
      {
        // In this case, the robot's controller would like to stabilize the robot with a QP controller.
        std::printf("Using Controller 2: QP Stabilizing Controller!\n");
        std::vector<double> qNew = StabilizingControllerContact(SimRobot, ActJacobians, ConeUnits, EdgeNumber, DOF, dt, qDesTraj, qdotDesTraj, qActTraj, qdotActTraj, RobotLinkInfo, RobotContactInfo, ContactPositionRef, ActContactPositions, ActVelocities, NumberOfContactPoints, 0);
        qDes = qNew;
      }
      break;
      default:
      {
      }
      break;
    }

    TrajAppender(StateTrajNames[0], qActTraj[qActTraj.size()-1], DOF);
    TrajAppender(StateTrajNames[1], qdotActTraj[qdotActTraj.size()-1], DOF);

    // Send the control command!
    Config qDesired(qDes);
    NewControllerPtr->SetConstant(qDesired);

    Sim.Advance(dt);
    Sim.UpdateModel();
    Backend.DoStateLogging_LinearPath(0, stateTrajFile_Name);

  }

  double KENow = SimRobot.GetKineticEnergy();
  int StepIndex = 0;
  while (KENow>=KETol)
  {
    // This loop is used to stabilize the robot
    SimSmoother(ControllerType, Sim, qdotDesTraj, DOF);
    SimRobot = *Sim.world->robots[0];
    KENow = SimRobot.GetKineticEnergy();

    /* Robot's COMPos and COMVel */
    Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
    CentroidalState(SimRobot, COMPos, COMVel);

    std::vector<Vector3>  ActContactPositions, ActVelocities;        // A vector of Vector3 points
    std::vector<Matrix>   ActJacobians;       // A vector of Jacobian matrices
    std::vector<int> ActStatus = ActContactNJacobian(SimRobot, RobotLinkInfo, RobotContactInfo, ActContactPositions, ActVelocities, ActJacobians, SDFInfo);
    std::vector<Vector3> ConeShiftedUnits, ConeUnits;
    ConeUnitGenerator(ActContactPositions, SDFInfo, ConeShiftedUnits, ConeUnits, EdgeNumber, mu);
    std::vector<Vector3> ProjActContactPos = ProjActContactPosGene(ActContactPositions);
    std::vector<PIPInfo> PIPSPTotal = PIPGenerator(ProjActContactPos, COMPos, COMVel);
    // int OEPIPIndex, CPPIPIndex;
    int CPPIPIndex;
    // double OEObjective = RBGenerator(PIPSPTotal, OEPIPIndex);
    double CPObjective = CapturePointGenerator(PIPSPTotal, CPPIPIndex);

    ContactPolytopeWriter(PIPSPTotal, EdgeFileNames);

    switch (CPPIPIndex)
    {
      case -1:
      {
        // Here is for stabilization
        /*  Controller Input  */
        switch (ControllerType)
        {
          case 1:
          {
            // In this case, the robot's controller holds a constant initial configuration.
            std::printf("Using Controller 1: Rigid-body Controller!\n");

            qActTraj.push_back(SimRobot.q);
            qdotActTraj.push_back(SimRobot.dq);
          }
          break;
          case 2:
          {
            // In this case, the robot's controller would like to stabilize the robot with a QP controller.
            std::printf("Using Controller 2: QP Stabilizing Controller!\n");
            std::vector<double> qNew = StabilizingControllerContact(SimRobot, ActJacobians, ConeUnits, EdgeNumber, DOF, dt, qDesTraj, qdotDesTraj, qActTraj, qdotActTraj, RobotLinkInfo, RobotContactInfo, ContactPositionRef, ActContactPositions, ActVelocities, NumberOfContactPoints, StepIndex);
            qDes = qNew;
          }
          break;
          default:
          {
          }
          break;
        }

      }
      break;
      default:
      {
        // Here is for robot's contact modification.
        std::printf("Critial PIP Index is %d\n", CPPIPIndex);

      }
      break;
    }
    TrajAppender(StateTrajNames[0], qActTraj[qActTraj.size()-1], DOF);
    TrajAppender(StateTrajNames[1], qdotActTraj[qdotActTraj.size()-1], DOF);

    // Send the control command!
    Config qDesired(qDes);
    NewControllerPtr->SetConstant(qDesired);

    Sim.Advance(dt);
    Sim.UpdateModel();
    Backend.DoStateLogging_LinearPath(0, stateTrajFile_Name);
  }

  return;
}
