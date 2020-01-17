#include <ctime>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <omp.h>
#include "Control/PathController.h"
#include "Simulation/WorldSimulation.h"
#include <ode/ode.h>

SignedDistanceFieldInfo NonlinearOptimizerInfo::SDFInfo;
std::vector<LinkInfo>   NonlinearOptimizerInfo::RobotLinkInfo;

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
  Robot SimRobot = *world.robots[0];

  /* 1. Load the Contact Link file */
  const std::string UserFilePath = FolderPath + "user/hrp2/";
  const std::string ContactLinkPath = UserFilePath + "ContactLink.txt";
  int NumberOfContactPoints;
  NonlinearOptimizerInfo::RobotLinkInfo = ContactInfoLoader(ContactLinkPath, NumberOfContactPoints);
  const std::string TorsoLinkFilePath = UserFilePath + "TorsoLink.txt";
  std::vector<int> TorsoLink = TorsoLinkReader(TorsoLinkFilePath);

  /* 2. Load the Contact Status file */
  const std::string ContactStatusPath = UserFilePath + "InitContact.txt";
  std::vector<ContactStatusInfo> RobotContactInfo = ContactStatusInfoLoader(ContactStatusPath);

  /* 3. Environment Geometry and Reachability Map*/
  const int GridsNo = 251;
  // NonlinearOptimizerInfo::SDFInfo = SignedDistanceFieldGene(world, GridsNo);
  NonlinearOptimizerInfo::SDFInfo = SignedDistanceFieldLoader(GridsNo);
  ReachabilityMap RMObject = ReachabilityMapGenerator(SimRobot, NonlinearOptimizerInfo::RobotLinkInfo, TorsoLink);

  /* 4. Robot State Loader */
  RobotConfigLoader(SimRobot, UserFilePath, "SampleTest.config");

  const int DOF = SimRobot.q.size();
  std::vector<double> InitRobotConfig(DOF), InitRobotVelocity(DOF, 0);
  std::vector<double> RobotConfigRef(DOF);
  for (int i = 0; i < DOF; i++)
  {
    RobotConfigRef[i] = SimRobot.q[i];
  }
  SimRobot.UpdateConfig(Config(RobotConfigRef));
  SimRobot.dq = InitRobotVelocity;

  /* 5. Initial Optimization Optimization (only for this project) */
  InitRobotConfig = InitialConfigurationOptimization(SimRobot, RobotContactInfo, RobotConfigRef);
  RobotConfigWriter(InitRobotConfig, UserFilePath, "InitConfig.config");
  SimRobot.UpdateConfig(Config(InitRobotConfig));
  SimRobot.UpdateGeometry();
  SimRobot.dq = InitRobotVelocity;
  bool SelfCollisionTest = SimRobot.SelfCollision();
  if(SelfCollisionTest == true)
  {
    std::cerr<<"Initial configuration failed due to self-collision!\n";
    exit(1);
  }

  //  Given the optimized result to be the initial state
  Sim.world->robots[0]->UpdateConfig(Config(InitRobotConfig));
  Sim.world->robots[0]->dq = InitRobotVelocity;

  Sim.controlSimulators[0].oderobot->SetConfig(Config(InitRobotConfig));
  Sim.controlSimulators[0].oderobot->SetVelocities(Config(InitRobotVelocity));

  /* 6. Projected Inverted Pendulum Plot */
  std::vector<Vector3> ActContactPositions, ActVelocities;
  std::vector<Matrix> ActJacobians;
  std::vector<int> ActStatus = ActContactNJacobian(SimRobot, NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfo, ActContactPositions, ActVelocities, ActJacobians, NonlinearOptimizerInfo::SDFInfo);

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
  double  TimeStep          = 0.025;
  int FileIndex = FileIndexFinder();
  string stateTrajFile = "stateTraj" + std::to_string(FileIndex) + ".path";
  const char *stateTrajFile_Name = stateTrajFile.c_str();
  
  while(Sim.time <= t_imp)
  {
    Sim.Advance(TimeStep);
    Sim.UpdateModel();
    StateTrajAppender(stateTrajFile_Name, Sim.time, SimRobot.q);
  }

  /* 8. Internal Experimentation */
  SimulationTest(Sim, NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfo, RMObject, TimeStep, FileIndex);

  return true;
}
