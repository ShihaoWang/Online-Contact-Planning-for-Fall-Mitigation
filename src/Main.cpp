#include <ctime>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <omp.h>
#include "Control/PathController.h"
#include "Simulation/WorldSimulation.h"
#include <ode/ode.h>

static bool InnerSimulation(const std::string & FolderPath, ViabilityKernelInfo & VKObj, const double & KEInit, const Vector3 & CentDirection)
{
  RobotWorld world;
  SimGUIBackend Backend(&world);
  WorldSimulation& Sim = Backend.sim;

  /* 0. Load the XML World file */
  string XMLFileStr = FolderPath + "/Envi4.xml";
  const char* XMLFile = XMLFileStr.c_str();    // Here we must give abstract path to the file
  if(!Backend.LoadAndInitSim(XMLFile))
  {
    std::cerr<<"World XML file path does not exist!"<<endl;
    return true;
  }

  /* 1. Load the Contact Link file */
  const std::string UserFilePath = FolderPath + "/user/hrp2/";
  const std::string ContactLinkPath = UserFilePath + "ContactLink.txt";
  int NumberOfContactPoints;
  std::vector<LinkInfo> RobotLinkInfo = ContactInfoLoader(ContactLinkPath, NumberOfContactPoints);

  /* 2. Load the Contact Status file */
  const std::string ContactStatusPath = UserFilePath + "InitContact.txt";
  std::vector<ContactStatusInfo> RobotContactInfo = ContactStatusInfoLoader(ContactStatusPath);

  /* 3. Signed Distance Field Computation */
  const int GridsNo = 251;
  // SignedDistanceFieldInfo SDFInfo = SignedDistanceFieldGene(world, GridsNo);
  SignedDistanceFieldInfo SDFInfo = SignedDistanceFieldLoader(GridsNo);

  std::vector<Config>  qTraj;
  std::vector<double> COMVelx, COMVely, COMVelz;
  Robot SimRobot = *world.robots[0];

  /* 4. Robot State Loader */

  // RobotConfigLoader(SimRobot, UserFilePath, "DefaultTest.config");
  // RobotConfigLoader(SimRobot, UserFilePath, "DefaultTester.config");
  // RobotConfigLoader(SimRobot, UserFilePath, "Exp0.config");
  RobotConfigLoader(SimRobot, UserFilePath, "Exp0_Load.config");
  // RobotConfigLoader(SimRobot, UserFilePath, "Exp1.config");
  // RobotConfigLoader(SimRobot, UserFilePath, "Exp1_Load.config");
  // RobotConfigLoader(SimRobot, UserFilePath, "Exp2.config");
  // RobotConfigLoader(SimRobot, UserFilePath, "Exp2_Load.config");
  // RobotConfigLoader(SimRobot, UserFilePath, "Exp3.config");
  // RobotConfigLoader(SimRobot, UserFilePath, "Exp3_Load.config");
  // RobotConfigLoader(SimRobot, UserFilePath, "Exp4.config");
  // RobotConfigLoader(SimRobot, UserFilePath, "Exp4_Load.config");

  std::vector<double> InitRobotConfig(SimRobot.q.size()), InitRobotVelocity(SimRobot.q.size()), ZeroRobotVelocity(SimRobot.q.size());
  std::vector<double> RobotConfigRef(SimRobot.q.size());
  for (int i = 0; i < SimRobot.q.size(); i++)
  {
    double scale = 3.0;
    InitRobotVelocity[i] = RandomValue(scale);
    RobotConfigRef[i] =   SimRobot.q[i];
    InitRobotConfig[i] =  SimRobot.q[i];
  }
  SimRobot.dq = InitRobotVelocity;

  /* 6. Initial State Optimization */
  bool ConfigOptFlag = false;
  bool VelocityOptFlag = true;
  bool InitFlag = InitialStateOptFn(SimRobot, RobotLinkInfo, RobotContactInfo, SDFInfo, RobotConfigRef, KEInit, CentDirection, InitRobotConfig, InitRobotVelocity, ConfigOptFlag, VelocityOptFlag);
  switch (InitFlag)
  {
    case false:
    {
      return false;
    }
    break;
    default:
    {
      printf("Initial Optimization Finished! \n");
    }
    break;
  }
  RobotConfigWriter(InitRobotConfig, UserFilePath, "InitConfig.config");

  SimRobot.UpdateConfig(Config(InitRobotConfig));
  SimRobot.dq = InitRobotVelocity;
  std::cout<<SimRobot.GetKineticEnergy()<<" J"<<std::endl;

  // std::vector<double> RobotConfig, RobotVelocity;
  // RobotStateLoader(UserFilePath, "TestConfig.config", "TestVelocity.config", RobotConfig, RobotVelocity);

  //  Given the optimized result to be the initial state
  Config InitRobotConfigNew(InitRobotConfig);
  Sim.world->robots[0]->UpdateConfig(InitRobotConfigNew);
  Sim.world->robots[0]->dq = ZeroRobotVelocity;

  Config InitRobotVelocityNew(ZeroRobotVelocity);
  Sim.controlSimulators[0].oderobot->SetConfig(InitRobotConfigNew);
  Sim.controlSimulators[0].oderobot->SetVelocities(InitRobotVelocityNew);

  /* 7. Projected Inverted Pendulum Plot*/
  std::vector<Vector3> ActContactPositions, ActVelocities;
  std::vector<Matrix> ActJacobians;
  std::vector<int> ActStatus = ActContactNJacobian(SimRobot, RobotLinkInfo, RobotContactInfo, ActContactPositions, ActVelocities, ActJacobians, SDFInfo);

  std::vector<Vector3> CPVertex, CPEdgeA, CPEdgeB;
  std::vector<FacetInfo> FacetInfoObj = ContactHullGeneration(ActContactPositions, CPVertex, CPEdgeA, CPEdgeB);      // This function output is only used for visualization purpose.
  ConvexEdgesWriter(FacetInfoObj, UserFilePath, "InitConfigCHEdges.txt");

  Vector3 InitCOM = SimRobot.GetCOM();
  //  std::vector<PIPInfo> PIPTotal = ContactEdgesGeneration(CPVertex,  CPEdgeA, CPEdgeB, InitCOM, InitCOM, SDFInfo);
  int FailureFlag;
  std::vector<PIPInfo> PIPTotal = ContactEdgesGenerationSP(ActContactPositions, ActVelocities, ActStatus, InitCOM, InitCOM, FailureFlag);   // SP denotes SP projection approach.
  PIPsWriter(PIPTotal, UserFilePath, "InitConfigPIPs.txt");
  std::vector<Vector3> FullPIPInters = FullPIPInterCal(FacetInfoObj, InitCOM);
  IntersectionsWriter(FullPIPInters, UserFilePath, "InitConfigIntersections.txt");

  double t_imp = 2.0;
  // Here first we would like to run the simulation for a certain amount of time to ensure the contact is well established.
  double  dt          = 0.025;
  int FileIndex = FileIndexFinder();
  string stateTrajFile = "stateTraj" + std::to_string(FileIndex) + ".path";
  const char *stateTrajFile_Name = stateTrajFile.c_str();
  while(Sim.time <= t_imp)
  {
    Sim.Advance(dt);
    Sim.UpdateModel();
    Backend.DoStateLogging_LinearPath(0, stateTrajFile_Name);
  }

  Sim.world->robots[0]->dq = InitRobotVelocity;
  Config InitRobotVelocityImpl(InitRobotVelocity);
  Sim.controlSimulators[0].oderobot->SetVelocities(InitRobotVelocityImpl);

  std::vector<Vector3> ActContactPositionsRef, ActVelocitiesRef;
  std::vector<Matrix> ActJacobiansRef;
  ActStatus = ActContactNJacobian(SimRobot, RobotLinkInfo, RobotContactInfo, ActContactPositionsRef, ActVelocitiesRef, ActJacobiansRef, SDFInfo);

  /* 8. Internal Experimentation */
  SimulationTest(Sim, VKObj, RobotLinkInfo, RobotContactInfo, SDFInfo, Backend, ActContactPositionsRef, CentDirection, dt, FileIndex);
  return true;
}

static void InitParaGenerator(double & KEInit, Vector3& CentDirection)
{
  // The robot's initial kinetic energy will be sampled from a distribution.
  // In addition, its centroidal direction will also be sampled.
  std::random_device rd;
  std::mt19937 gen(rd());
  double KELow = 0.0;
  double KEUpp = 30.0;
  std::uniform_real_distribution<> KEDis(KELow, KEUpp);
  KEInit = KEDis(gen);

  double xLimit, yLimit, zLimit;

  // 2 Contact:
  // xLimit = 0.2;  yLimit = 0.35;  zLimit = 0.1;       // Case 1 to Case 4
  xLimit = 0.1;  yLimit = 0.1;  zLimit = 0.1;

  std::uniform_real_distribution<> xDirectionDis(-xLimit, xLimit);
  std::uniform_real_distribution<> yDirectionDis(-yLimit, yLimit);
  std::uniform_real_distribution<> zDirectionDis(-zLimit, zLimit);

  double xDirectionInit = xDirectionDis(gen);
  double yDirectionInit = yDirectionDis(gen);
  double zDirectionInit = zDirectionDis(gen);

  double DirectionInitNorm = sqrt(xDirectionInit * xDirectionInit + yDirectionInit * yDirectionInit + zDirectionInit * zDirectionInit);
  DirectionInitNorm = 1.0;
  CentDirection.x = xDirectionInit/DirectionInitNorm;
  CentDirection.y = yDirectionInit/DirectionInitNorm;
  CentDirection.z = zDirectionInit/DirectionInitNorm;

  return;
}

int main()
{
  string ViabilityKernelPath = "/home/motion/Desktop/VKACC/build/";
  bool VKFastFlag = false;
  ViabilityKernelInfo VKObj = ViabilityKernelDataLoader(ViabilityKernelPath, VKFastFlag);
  for (int i = 0; i < 101; i++)
  {
    if(i%5==0)
    {
      string mv_command = "mv -f *Traj*.* ./ExpData";
      const char *mv_command_str = mv_command.c_str();
      std::system(mv_command_str);

      mv_command = "mv -f Specs*.* ./ExpData";
      mv_command_str = mv_command.c_str();
      std::system(mv_command_str);
    }
    // Simulation loop at each time
    std::string FolderPath = "/home/motion/Desktop/Stabilizability-Analysis-with-Polytopic-Viability-Kernel";
    bool InitializationFlag = false;
    while (InitializationFlag == false)
    {
      double KEInit;
      Vector3 CentDirection;
      InitParaGenerator(KEInit, CentDirection);   // Here Cent Direction stands for the initial centroidal velocity direction!
      KEInit = 15.0;
      InitializationFlag = InnerSimulation(FolderPath, VKObj, KEInit, CentDirection);
    }
  }
  return 0;
}
