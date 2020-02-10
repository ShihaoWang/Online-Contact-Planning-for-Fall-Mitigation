#include <ctime>
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <omp.h>
#include "Control/PathController.h"
#include "Simulation/WorldSimulation.h"
#include <ode/ode.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string>

SignedDistanceFieldInfo NonlinearOptimizerInfo::SDFInfo;
std::vector<LinkInfo>   NonlinearOptimizerInfo::RobotLinkInfo;
bool SDFFlag = false;
bool RMFlag = false;

int main()
{
  /* 1. Load the Contact Link file */
  const std::string UserFilePath = "../user/hrp2/";
  const std::string ContactLinkPath = UserFilePath + "ContactLink.txt";
  int NumberOfContactPoints;
  NonlinearOptimizerInfo::RobotLinkInfo = ContactInfoLoader(ContactLinkPath, NumberOfContactPoints);
  const std::string TorsoLinkFilePath = UserFilePath + "TorsoLink.txt";
  std::vector<int> TorsoLink = TorsoLinkReader(TorsoLinkFilePath);

  /* 2. Load the Envi and Contact Status file */
  std::ifstream FolderPathFile("./Specs/EnviSpecs.txt");
  std::string EnviName;
  std::string ExpName;
  std::string ContactType;
  std::getline(FolderPathFile, EnviName);
  std::getline(FolderPathFile, ExpName);
  std::getline(FolderPathFile, ContactType);
  FolderPathFile.close();

  const std::string SpecificPath = "../result/" + ExpName + "/" + ContactType;

  /* 3. Environment Geometry and Reachability Map */
  const int GridsNo = 251;
  struct stat buffer;   // This is used to check whether "SDFSpecs.bin" exists or not.
  const string SDFSpecsName = "SDFSpecs.bin";
  if(stat (SDFSpecsName.c_str(), &buffer) == 0)
  {
    NonlinearOptimizerInfo::SDFInfo = SignedDistanceFieldLoader(GridsNo);
  }
  else
  {
    if(!SDFFlag)
    {
      RobotWorld worldObj;
      SimGUIBackend Backend(&worldObj);
      WorldSimulation& Sim = Backend.sim;

      string XMLFileStr =  "../" + EnviName;
      const char* XMLFile = XMLFileStr.c_str();    // Here we must give abstract path to the file
      if(!Backend.LoadAndInitSim(XMLFile))
      {
        std::cerr<< EnviName<<" file does not exist in that path!"<<endl;
        return -1;
      }
      NonlinearOptimizerInfo::SDFInfo = SignedDistanceFieldGene(worldObj, GridsNo);
      SDFFlag = true;
    }
  }

  /* 3. Reachability Map Generation */
  ReachabilityMap RMObject;
  if(!RMFlag)
  {
    RobotWorld worldObj;
    SimGUIBackend Backend(&worldObj);
    WorldSimulation& Sim = Backend.sim;

    string XMLFileStr =  "../" + EnviName;
    const char* XMLFile = XMLFileStr.c_str();    // Here we must give abstract path to the file
    if(!Backend.LoadAndInitSim(XMLFile))
    {
      std::cerr<< EnviName<<" file does not exist in that path!"<<endl;
      return -1;
    }
    RMObject = ReachabilityMapGenerator(*worldObj.robots[0], NonlinearOptimizerInfo::RobotLinkInfo, TorsoLink);
  }

  int FileIndex = FileIndexFinder(false);
  int TotalNumber = 100;

  /* 4. Internal Experimentation Loop */
  while(FileIndex<=TotalNumber)
  {
    // Let them be internal objects
    string str = "cd " + SpecificPath + "/" + std::to_string(FileIndex) + "/";
    str+="&& rm -f *Traj.txt && rm -f *.path && rm -f *InfoFile.txt && rm -f PlanTime.txt";
    const char *command = str.c_str();
    system(command);

    RobotWorld world;
    SimGUIBackend Backend(&world);
    WorldSimulation& Sim = Backend.sim;

    string XMLFileStr = "../" + EnviName;
    const char* XMLFile = XMLFileStr.c_str();    // Here we must give abstract path to the file
    if(!Backend.LoadAndInitSim(XMLFile))
    {
      std::cerr<< EnviName << " file does not exist in that path!" << endl;
      return -1;
    }
    Robot SimRobot = *world.robots[0];
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
    bool SimFlag = SimulationTest(Sim, NonlinearOptimizerInfo::RobotLinkInfo, RobotContactInfo, RMObject, SpecificPath, FileIndex);
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
