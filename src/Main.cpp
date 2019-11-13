#include <ctime>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <omp.h>
#include "Control/PathController.h"
#include "Simulation/WorldSimulation.h"
#include <ode/ode.h>

static void InitParaGenerator(double & KEInit, Vector3& CentDirection)
{
  // The robot's initial kinetic energy will be sampled from a distribution.
  // In addition, its centroidal direction will also be sampled.
  std::random_device rd;
  std::mt19937 gen(rd());

  // Scenario 1
  double KELow = 0.0;
  double KEUpp = 50.0;

  std::uniform_real_distribution<> KEDis(KELow, KEUpp);
  KEInit = KEDis(gen);

  // These three terms will not be changed.
  double xLimit = 0.25;
  double yLimit = 0.25;
  double zLimit = 0.25;

  std::uniform_real_distribution<> xDirectionDis(-xLimit, xLimit);
  std::uniform_real_distribution<> yDirectionDis(-yLimit, yLimit);
  std::uniform_real_distribution<> zDirectionDis(-zLimit, zLimit);

  double xDirectionInit = xDirectionDis(gen);
  double yDirectionInit = yDirectionDis(gen);
  double zDirectionInit = zDirectionDis(gen);

  double DirectionInitNorm = sqrt(xDirectionInit * xDirectionInit + yDirectionInit * yDirectionInit + zDirectionInit * zDirectionInit);
  CentDirection.x = xDirectionInit/DirectionInitNorm;
  CentDirection.y = yDirectionInit/DirectionInitNorm;
  CentDirection.z = zDirectionInit/DirectionInitNorm;

  return;
}

int main()
{
  /* 0. Load the XML World file */
  RobotWorld world;
  SimGUIBackend Backend(&world);
  WorldSimulation& Sim = Backend.sim;

  std::string FolderPath = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/";
  std::string EnviName = "Envi1.xml";

  string XMLFileStr = FolderPath + EnviName;
  const char* XMLFile = XMLFileStr.c_str();    // Here we must give abstract path to the file
  if(!Backend.LoadAndInitSim(XMLFile))
  {
    std::cerr<< EnviName<<" file does not exist in that path!"<<endl;
    return -1;
  }

  /* 1. Load the Contact Link file */
  const std::string UserFilePath = FolderPath + "user/hrp2/";
  const std::string ContactLinkPath = UserFilePath + "ContactLink.txt";
  int NumberOfContactPoints;
  std::vector<LinkInfo> RobotLinkInfo = ContactInfoLoader(ContactLinkPath, NumberOfContactPoints);

  /* 2. Load the Contact Status file */
  const std::string ContactStatusPath = UserFilePath + "InitContact.txt";
  std::vector<ContactStatusInfo> RobotContactInfo = ContactStatusInfoLoader(ContactStatusPath);

  /* 3. Environment Geometry */
  Meshing::PointCloud3D EnviPointCloud = PointCloudGene(world);
  const int GridsNo = 251;
  SignedDistanceFieldInfo SDFInfo = SignedDistanceFieldGene(world, GridsNo);
  // SignedDistanceFieldInfo SDFInfo = SignedDistanceFieldLoader(GridsNo);

  std::map<int, std::vector<RMPoint>> asdf = ReachabilityMapGenerator();

  /* 4. Robot State Loader */
  Robot SimRobot = *world.robots[0];
  RobotConfigLoader(SimRobot, UserFilePath, "SampleTest.config");

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

  /* 5. Initial State Optimization */
  double KEInit;
  Vector3 CentDirection;
  InitParaGenerator(KEInit, CentDirection);
  bool ConfigOptFlag = true;
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


  //  Given the optimized result to be the initial state
  Config InitRobotConfigNew(InitRobotConfig);
  Sim.world->robots[0]->UpdateConfig(InitRobotConfigNew);
  Sim.world->robots[0]->dq = ZeroRobotVelocity;

  Config InitRobotVelocityNew(ZeroRobotVelocity);
  Sim.controlSimulators[0].oderobot->SetConfig(InitRobotConfigNew);
  Sim.controlSimulators[0].oderobot->SetVelocities(InitRobotVelocityNew);

  /* 6. Projected Inverted Pendulum Plot */
  std::vector<Vector3> ActContactPositions, ActVelocities;
  std::vector<Matrix> ActJacobians;
  std::vector<int> ActStatus = ActContactNJacobian(SimRobot, RobotLinkInfo, RobotContactInfo, ActContactPositions, ActVelocities, ActJacobians, SDFInfo);

  std::vector<Vector3> CPVertex, CPEdgeA, CPEdgeB;
  std::vector<FacetInfo> FacetInfoObj = ContactHullGeneration(ActContactPositions, CPVertex, CPEdgeA, CPEdgeB);      // This function output is only used for visualization purpose.
  ConvexEdgesWriter(FacetInfoObj, UserFilePath, "InitConfigCHEdges.txt");

  Vector3 InitCOM = SimRobot.GetCOM();
  std::vector<PIPInfo> PIPTotal = PIPGenerator(ActContactPositions, InitCOM, InitCOM);   // SP denotes SP projection approach.
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

  // // It depends on what sort of disturbances to be added to the robot!
  // Sim.world->robots[0]->dq = InitRobotVelocity;
  // Config InitRobotVelocityImpl(InitRobotVelocity);
  // Sim.controlSimulators[0].oderobot->SetVelocities(InitRobotVelocityImpl);

  std::vector<Vector3> ActContactPositionsRef, ActVelocitiesRef;
  std::vector<Matrix> ActJacobiansRef;
  ActStatus = ActContactNJacobian(SimRobot, RobotLinkInfo, RobotContactInfo, ActContactPositionsRef, ActVelocitiesRef, ActJacobiansRef, SDFInfo);

  /* 8. Internal Experimentation */
  SimulationTest(Sim, RobotLinkInfo, RobotContactInfo, SDFInfo, Backend, ActContactPositionsRef, dt, FileIndex);

  return true;
}
