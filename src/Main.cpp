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
  std::string FolderPath = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/";
  std::string EnviName = "Envi1.xml";

  /* 1. Load the Contact Link file */
  const std::string UserFilePath = FolderPath + "user/hrp2/";
  const std::string ContactLinkPath = UserFilePath + "ContactLink.txt";
  int NumberOfContactPoints;
  NonlinearOptimizerInfo::RobotLinkInfo = ContactInfoLoader(ContactLinkPath, NumberOfContactPoints);
  const std::string TorsoLinkFilePath = UserFilePath + "TorsoLink.txt";
  std::vector<int> TorsoLink = TorsoLinkReader(TorsoLinkFilePath);

  /* 2. Load the Contact Status file */
  const std::string ExpName = "flat/";
  const std::string ContactType = "1Contact/";
  const std::string SpecificPath = FolderPath + "result/" + ExpName + ContactType;

  /* 3. Environment Geometry and Reachability Map*/
  const int GridsNo = 251;
  // NonlinearOptimizerInfo::SDFInfo = SignedDistanceFieldGene(world, GridsNo);
  NonlinearOptimizerInfo::SDFInfo = SignedDistanceFieldLoader(GridsNo);

  /*  5. Load Impulse Force Magnitude*/
  Vector3 IFMax = ImpulForceMaxReader(SpecificPath, "ImpulseForce.txt");

  /* 6. Internal Experimentation Loop*/
  int FileIndex = 3;
  int TotalNumber = 10;

  while(FileIndex<=TotalNumber)
  {
    // Let them be internal objects
    string str = "cd " + SpecificPath + std::to_string(FileIndex) + "/";
    str+="&& rm -f *Traj.txt && rm -f *.path && rm -f *InfoFile.txt && rm -f PlanTime.txt";
    str+=" && cd /home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/user/hrp2/ && rm -f *Opt*.config";
    const char *command = str.c_str();
    system(command);

    RobotWorld world;
    SimGUIBackend Backend(&world);
    WorldSimulation& Sim = Backend.sim;

    string XMLFileStr = FolderPath + EnviName;
    const char* XMLFile = XMLFileStr.c_str();    // Here we must give abstract path to the file
    if(!Backend.LoadAndInitSim(XMLFile))
    {
      std::cerr<< EnviName<<" file does not exist in that path!"<<endl;
      return -1;
    }
    Robot SimRobot = *world.robots[0];
    ReachabilityMap RMObject = ReachabilityMapGenerator(SimRobot, NonlinearOptimizerInfo::RobotLinkInfo, TorsoLink);

    RobotConfigLoader(SimRobot, SpecificPath + std::to_string(FileIndex) + "/", "InitConfig.config");

    const std::string ContactStatusPath = SpecificPath + std::to_string(FileIndex) + "/ContactStatus.txt";
    std::vector<ContactStatusInfo> RobotContactInfo = ContactStatusInfoLoader(ContactStatusPath);

    std::vector<double> InitRobotConfig(SimRobot.q);
    std::vector<double> InitRobotVelocity(SimRobot.q.size(), 0.0);
    std::vector<double> RobotConfigRef = InitRobotVelocity;

    //  Given the optimized result to be the initial state
    Sim.world->robots[0]->UpdateConfig(Config(InitRobotConfig));
    Sim.world->robots[0]->dq = InitRobotVelocity;

    Sim.controlSimulators[0].oderobot->SetConfig(Config(InitRobotConfig));
    Sim.controlSimulators[0].oderobot->SetVelocities(Config(InitRobotVelocity));

    bool SimFlag = SimulationTest(Sim, NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfo, RMObject, SpecificPath, FileIndex, IFMax);
    switch (SimFlag)
    {
      case false:
      break;
      default:
      FileIndex++;
      break;
    }
  }
  return 1;
}
