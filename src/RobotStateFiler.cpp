// This function is used to load in the robot's specification
#include <iostream>
#include <fstream>
#include <sstream>
#include <Modeling/Robot.h>
#include "CommonHeader.h"

using namespace std;

void RobotConfigLoader(Robot &SimRobot, const string &user_path, const string &file_name)
{
  string str_line, str_keyword;
  str_keyword = "\t";
  int flag = 0;

  string config_file_path = user_path + file_name;
  ifstream ConfigInfofile (config_file_path);

  std::vector<double> RobotConfig, RobotVelocity;
  if (ConfigInfofile.is_open())
  {
    while (getline (ConfigInfofile,str_line) )
    {
      size_t start_pos = str_line.find(str_keyword);        // Here gives out the position of the \t
      if (start_pos != string::npos)
      {
        string DOF_str ="";
        for (size_t j = 0; j < start_pos; j++)
        {
          DOF_str+= str_line[j];
        }
        const int DOF = stoi(DOF_str);
        RobotConfig.reserve(DOF);
        RobotVelocity.reserve(DOF);
        str_line.erase(0,start_pos+1);

        std::istringstream ss(str_line);
        string Config_i;
        while (ss >> Config_i)
        {
          RobotConfig.push_back(std::stod(Config_i));
          RobotVelocity.push_back(0);
        }
      }
      else
      {
        std::cout<<"Wrong! .config file cannot be found!"<<endl;
      }
    }
    ConfigInfofile.close();
    flag = 1;
  }
  else cout << "Unable to open file";

  // Update the SimRobot's status
  RobotStateToSimRobot(SimRobot, RobotConfig, RobotVelocity);
  return;
}

void RobotConfigWriter(const std::vector<double> & Config, const string &user_path, const string &config_file_name)
{
  std::ofstream ConfigInfoFile;
  std::string config_file_path = user_path + config_file_name;
  ConfigInfoFile.open (config_file_path);
  ConfigInfoFile<<std::to_string(Config.size())<<"\t";
  for (int i = 0; i < Config.size(); i++)
  {
    ConfigInfoFile << std::to_string(Config[i])<<" ";
  }
  ConfigInfoFile.close();
}

void RobotStateLoader(Robot &SimRobot, const string &user_path, const string &config_file_name, const string &velo_file_name)
{
  string str_line;
  string config_file_path = user_path + config_file_name;
  ifstream ConfigInfofile (config_file_path);
  std::vector<double> RobotConfig, RobotVelocity;
  if (ConfigInfofile.is_open())
  {
    while (getline (ConfigInfofile, str_line) )
    {
      RobotConfig.push_back(stod(str_line));
    }
    ConfigInfofile.close();
  }
  else
  {
    std::cout<<"Wrong! config txt cannot be found!"<<endl;
  }

  string velocity_file_path = user_path + velo_file_name;
  ifstream VelocityInfofile (velocity_file_path);
  if (VelocityInfofile.is_open())
  {
    while (getline (VelocityInfofile, str_line) )
    {
      RobotVelocity.push_back(stod(str_line));
    }
    VelocityInfofile.close();
  }
  else
  {
    std::cout<<"Wrong! velocity txt cannot be found!"<<endl;
  }
  RobotStateToSimRobot(SimRobot, RobotConfig, RobotVelocity);
  return;
}

void IntersectionsWriter(const std::vector<Vector3> & Intersections, const string &user_path, const string &inters_file_name)
{
  // This function is used to write the 3d intersection into file
  string str_line;
  string inters_file_path = user_path + inters_file_name;
  ofstream Intersectionfile (inters_file_path);

  for (int i = 0; i < Intersections.size(); i++)
  {
    Intersectionfile<<std::to_string(Intersections[i].x)<<" "<<std::to_string(Intersections[i].y)<<" "<<std::to_string(Intersections[i].z)<<"\n";
  }
  Intersectionfile.close();
}

void RobotStateWriter(const std::vector<double> &Config, const std::vector<double> &Velocity, const string &user_path, const string &config_file_name, const string &velo_file_name)
{
  string str_line;
  string config_file_path = user_path + config_file_name;
  ofstream ConfigInfofile (config_file_path);

  for (int i = 0; i < Config.size(); i++)
  {
    ConfigInfofile<<std::to_string(Config[i])<<"\n";
  }
  ConfigInfofile.close();

  string velocity_file_path = user_path + velo_file_name;
  ofstream VelocityInfofile (velocity_file_path);

  for (int i = 0; i < Velocity.size(); i++)
  {
    VelocityInfofile<<std::to_string(Velocity[i])<<"\n";
  }
  VelocityInfofile.close();
}

void PIPsWriter(const std::vector<PIPInfo>& PIPInfoTotal, const string &user_path, const string &edge_file_name)
{
  // This function is used to write the convex Edges to files for visualization
  string str_line;
  string edge_file_path = user_path + edge_file_name;
  ofstream PIPfile (edge_file_path);

  for (int i = 0; i < PIPInfoTotal.size(); i++)
  {
    // Each element in EdgeVertices is a pair of Vector3 element
    PIPfile<<std::to_string(PIPInfoTotal[i].EdgeA.x)<<" "<<std::to_string(PIPInfoTotal[i].EdgeA.y)<<" "<<std::to_string(PIPInfoTotal[i].EdgeA.z)<<"\n";
    PIPfile<<std::to_string(PIPInfoTotal[i].EdgeB.x)<<" "<<std::to_string(PIPInfoTotal[i].EdgeB.y)<<" "<<std::to_string(PIPInfoTotal[i].EdgeB.z)<<"\n";
    PIPfile<<std::to_string(PIPInfoTotal[i].Intersection.x)<<" "<<std::to_string(PIPInfoTotal[i].Intersection.y)<<" "<<std::to_string(PIPInfoTotal[i].Intersection.z)<<"\n";
    PIPfile<<std::to_string(PIPInfoTotal[i].x_prime_unit.x)<<" "<<std::to_string(PIPInfoTotal[i].x_prime_unit.y)<<" "<<std::to_string(PIPInfoTotal[i].x_prime_unit.z)<<"\n";
    PIPfile<<std::to_string(PIPInfoTotal[i].y_prime_unit.x)<<" "<<std::to_string(PIPInfoTotal[i].y_prime_unit.y)<<" "<<std::to_string(PIPInfoTotal[i].y_prime_unit.z)<<"\n";
    PIPfile<<std::to_string(PIPInfoTotal[i].z_prime_unit.x)<<" "<<std::to_string(PIPInfoTotal[i].z_prime_unit.y)<<" "<<std::to_string(PIPInfoTotal[i].z_prime_unit.z)<<"\n";
  }
  PIPfile.close();
}

void ConvexEdgesWriter(const std::vector<FacetInfo>& FacetInfoObj, const string &user_path, const string &edge_file_name)
{
  // This function is used to write the convex Edges to files for visualization
  string str_line;
  string edge_file_path = user_path + edge_file_name;
  ofstream ConvexEdgefile (edge_file_path);

  for (int i = 0; i < FacetInfoObj.size(); i++)
  {
    for (int j = 0; j < FacetInfoObj[i].FacetEdges.size(); j++)
    {
      // Each element in EdgeVertices is a pair of Vector3 element
      ConvexEdgefile<<std::to_string(FacetInfoObj[i].FacetEdges[j].first.x)<<" "<<std::to_string(FacetInfoObj[i].FacetEdges[j].first.y)<<" "<<std::to_string(FacetInfoObj[i].FacetEdges[j].first.z)<<"\n";
      ConvexEdgefile<<std::to_string(FacetInfoObj[i].FacetEdges[j].second.x)<<" "<<std::to_string(FacetInfoObj[i].FacetEdges[j].second.y)<<" "<<std::to_string(FacetInfoObj[i].FacetEdges[j].second.z)<<"\n";
    }
  }
  ConvexEdgefile.close();
}

void VectorWriter(const std::vector<double> & Cost_Vec, const string &user_path, const string &config_file_name)
{
  string str_line;
  string config_file_path = user_path + config_file_name;
  ofstream Vectorfile (config_file_path);

  for (int i = 0; i < Cost_Vec.size(); i++)
  {
    Vectorfile<<std::to_string(Cost_Vec[i])<<"\n";
  }
  Vectorfile.close();
}

void TrajAppender(const char * qTrajFile_Name, const Config & Traj_i, const int & DOF)
{
  // This following part is used to save the robot's desired position, velocity and acceleration trajectory
  std::ofstream TrajFileWriter;
  TrajFileWriter.open(qTrajFile_Name, std::ios_base::app);
  for (int i = 0; i < DOF; i++)
  {
    TrajFileWriter<<std::to_string(Traj_i[i])<<" ";
  }
  TrajFileWriter<<"\n";
  TrajFileWriter.close();
}

void SpecsWriter(const Robot & SimRobot, const double & t_final, const double & dt, const int & InitContactNo, const int & FileIndex)
{
  // This function is used to write Specs vector into file.
  std::vector<double> Specs;
  Specs.push_back(t_final);
  Specs.push_back(dt);
  for (int i = 0; i < SimRobot.q.size(); i++)
  {
    Specs.push_back(SimRobot.q[i]);
  }
  for (int i = 0; i < SimRobot.dq.size(); i++)
  {
    Specs.push_back(SimRobot.dq[i]);
  }
  Vector3 COM(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(SimRobot, COM, COMVel);

  Specs.push_back(COMVel.x);
  Specs.push_back(COMVel.y);
  Specs.push_back(COMVel.z);
  Specs.push_back(SimRobot.GetKineticEnergy());

  Specs.push_back(InitContactNo);

  // File Name according to string
  string SpecsFileStr = "Specs" + std::to_string(FileIndex) + ".txt";
  const char *SpecsFile_Name = SpecsFileStr.c_str();

  std::ofstream SpecsFile;
  SpecsFile.open(SpecsFile_Name, std::ios_base::app);
  for (int i = 0; i < Specs.size(); i++)
  {
    SpecsFile<<std::to_string(Specs[i])<<"\n";
  }
  SpecsFile.close();
}

void COMDesWriter(const int & FileIndex, const Vector3 & COMPosdes)
{
  // This function is used to write desired center of mass vector into a file
  const string COMxdesFile = "COMxdesTraj" + std::to_string(FileIndex) + ".txt";
  const char *COMxdesFile_Name = COMxdesFile.c_str();

  const string COMydesFile = "COMydesTraj" + std::to_string(FileIndex) + ".txt";
  const char *COMydesFile_Name = COMydesFile.c_str();

  const string COMzdesFile = "COMzdesTraj" + std::to_string(FileIndex) + ".txt";
  const char *COMzdesFile_Name = COMzdesFile.c_str();

  std::ofstream COMxdesFileWriter;
  COMxdesFileWriter.open(COMxdesFile_Name, std::ios_base::app);
  COMxdesFileWriter<<std::to_string(COMPosdes.x)<<"\n";
  COMxdesFileWriter.close();

  std::ofstream COMydesFileWriter;
  COMydesFileWriter.open(COMydesFile_Name, std::ios_base::app);
  COMydesFileWriter<<std::to_string(COMPosdes.y)<<"\n";

  COMydesFileWriter.close();

  std::ofstream COMzdesFileWriter;
  COMzdesFileWriter.open(COMzdesFile_Name, std::ios_base::app);
  COMzdesFileWriter<<std::to_string(COMPosdes.z)<<"\n";
  COMzdesFileWriter.close();

  return;
}

void CentroidalFailureMetricWriter(const Vector3 & COM, const Vector3 & COMVel, const double & KE, const std::vector<double> FailureMetricVec, const std::vector<const char*> & CentroidalFileNames, const std::vector<const char*> & FailureMetricNames)
{
  // This function is used to write centroidal trajectories and faliure metric into files.
   double PVKRBTraj_i =   FailureMetricVec[0];
   double PVKCPTraj_i =   FailureMetricVec[1];
   double PVKHJBTraj_i =  FailureMetricVec[2];
   double ZSCTraj_i =     FailureMetricVec[3];
   double OETraj_i =      FailureMetricVec[4];
   double CPTraj_i =      FailureMetricVec[5];
   double ZMPTraj_i =     FailureMetricVec[6];

  // Write all the function values
  std::ofstream COMxFileWriter;
  COMxFileWriter.open(CentroidalFileNames[0], std::ios_base::app);
  COMxFileWriter<<std::to_string(COM.x)<<"\n";
  COMxFileWriter.close();

  std::ofstream COMyFileWriter;
  COMyFileWriter.open(CentroidalFileNames[1], std::ios_base::app);
  COMyFileWriter<<std::to_string(COM.y)<<"\n";
  COMyFileWriter.close();

  std::ofstream COMzFileWriter;
  COMzFileWriter.open(CentroidalFileNames[2], std::ios_base::app);
  COMzFileWriter<<std::to_string(COM.z)<<"\n";
  COMzFileWriter.close();

  std::ofstream COMVelxFileWriter;
  COMVelxFileWriter.open(CentroidalFileNames[3], std::ios_base::app);
  COMVelxFileWriter<<std::to_string(COMVel.x)<<"\n";
  COMVelxFileWriter.close();

  std::ofstream COMVelyFileWriter;
  COMVelyFileWriter.open(CentroidalFileNames[4], std::ios_base::app);
  COMVelyFileWriter<<std::to_string(COMVel.y)<<"\n";
  COMVelyFileWriter.close();

  std::ofstream COMVelzFileWriter;
  COMVelzFileWriter.open(CentroidalFileNames[5], std::ios_base::app);
  COMVelzFileWriter<<std::to_string(COMVel.z)<<"\n";
  COMVelzFileWriter.close();

  std::ofstream KEFileWriter;
  KEFileWriter.open(CentroidalFileNames[6], std::ios_base::app);
  KEFileWriter<<std::to_string(KE)<<"\n";
  KEFileWriter.close();

  std::ofstream PVKRBFileWriter;
  PVKRBFileWriter.open(FailureMetricNames[0], std::ios_base::app);
  PVKRBFileWriter<<std::to_string(PVKRBTraj_i)<<"\n";
  PVKRBFileWriter.close();

  std::ofstream PVKCPFileWriter;
  PVKCPFileWriter.open(FailureMetricNames[1], std::ios_base::app);
  PVKCPFileWriter<<std::to_string(PVKCPTraj_i)<<"\n";
  PVKCPFileWriter.close();

  std::ofstream PVKHJBFileWriter;
  PVKHJBFileWriter.open(FailureMetricNames[2], std::ios_base::app);
  PVKHJBFileWriter<<std::to_string(PVKHJBTraj_i)<<"\n";
  PVKHJBFileWriter.close();

  std::ofstream ZSCFileWriter;
  ZSCFileWriter.open(FailureMetricNames[3], std::ios_base::app);
  ZSCFileWriter<<std::to_string(ZSCTraj_i)<<"\n";
  ZSCFileWriter.close();

  std::ofstream OEFileWriter;
  OEFileWriter.open(FailureMetricNames[4], std::ios_base::app);
  OEFileWriter<<std::to_string(OETraj_i)<<"\n";
  OEFileWriter.close();

  std::ofstream CPFileWriter;
  CPFileWriter.open(FailureMetricNames[5], std::ios_base::app);
  CPFileWriter<<std::to_string(CPTraj_i)<<"\n";
  CPFileWriter.close();

  std::ofstream ZMPFileWriter;
  ZMPFileWriter.open(FailureMetricNames[6], std::ios_base::app);
  ZMPFileWriter<<std::to_string(ZMPTraj_i)<<"\n";
  ZMPFileWriter.close();
}

void RobotStateLoader(const string &user_path, const string &config_file_name, const string &velo_file_name, std::vector<double> & RobotConfig, std::vector<double> & RobotVelocity)
{
  string str_line;
  string config_file_path = user_path + config_file_name;
  ifstream ConfigInfofile (config_file_path);
  if (ConfigInfofile.is_open())
  {
    while (getline (ConfigInfofile, str_line) )
    {
      RobotConfig.push_back(stod(str_line));
    }
    ConfigInfofile.close();
  }
  else
  {
    std::cout<<"Wrong! config txt cannot be found!"<<endl;
  }

  string velocity_file_path = user_path + velo_file_name;
  ifstream VelocityInfofile (velocity_file_path);
  if (VelocityInfofile.is_open())
  {
    while (getline (VelocityInfofile, str_line) )
    {
      RobotVelocity.push_back(stod(str_line));
    }
    VelocityInfofile.close();
  }
  else
  {
    std::cout<<"Wrong! velocity txt cannot be found!"<<endl;
  }
  return;
}
