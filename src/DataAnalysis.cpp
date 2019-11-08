#include "RobotInfo.h"
#include "CommonHeader.h"
#include <setoper.h>
#include <cdd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <bits/stdc++.h>
#include <sstream>
#include <iterator>

static int     EdgeNumber      = 4;
static double  mu              = 0.5;

struct FallPnAInfo
{
  FallPnAInfo()
  {

  };
  FallPnAInfo(double _RBAccu, double _RBAccuFall, double _HJBAccu, double _HJBAccuFall, double _CPCEAccu, double _CPCEAccuFall, double _CPSPAccu, double _CPSPAccuFall, double _ZSCAccu, double _ZSCAccuFall)
  {
    RBAccu = _RBAccu;
    RBAccuFall= _RBAccuFall;

    HJBAccu = _HJBAccu;
    HJBAccuFall = _HJBAccuFall;

    CPCEAccu = _CPCEAccu;
    CPCEAccuFall = _CPCEAccuFall;

    CPSPAccu = _CPSPAccu;
    CPSPAccuFall = _CPSPAccuFall;

    ZSCAccu = _ZSCAccu;
    ZSCAccuFall = _ZSCAccuFall;
  }
  double RBAccu;            double RBAccuFall;
  double HJBAccu;           double HJBAccuFall;
  double CPCEAccu;          double CPCEAccuFall;
  double CPSPAccu;          double CPSPAccuFall;
  double ZSCAccu;           double ZSCAccuFall;
};

static void FallPnAInfoTotWriter(const std::vector<FallPnAInfo> & FallPnAInfoTot, const int & CaseNumber)
{

  // This function is used to generate the ROC curve
  string RBAccTrajFile = std::to_string(CaseNumber) + "_RB_Acc.txt";
  const char *RBAccTrajFile_Name = RBAccTrajFile.c_str();
  std::ofstream RBAccWriter;
  RBAccWriter.open(RBAccTrajFile_Name);
  for (int i = 0; i < FallPnAInfoTot.size(); i++)
  {
    RBAccWriter<<std::to_string(FallPnAInfoTot[i].RBAccu)<<"\n";
  }
  RBAccWriter.close();

  string RBPreTrajFile = std::to_string(CaseNumber) + "_RB_Pre.txt";
  const char *RBPreTrajFile_Name = RBPreTrajFile.c_str();
  std::ofstream RBPreWriter;
  RBPreWriter.open(RBPreTrajFile_Name);
  for (int i = 0; i < FallPnAInfoTot.size(); i++)
  {
    RBPreWriter<<std::to_string(FallPnAInfoTot[i].RBAccuFall)<<"\n";
  }
  RBPreWriter.close();

  string HJBAccTrajFile = std::to_string(CaseNumber) + "_HJB_Acc.txt";
  const char *HJBAccTrajFile_Name = HJBAccTrajFile.c_str();
  std::ofstream HJBAccWriter;
  HJBAccWriter.open(HJBAccTrajFile_Name);
  for (int i = 0; i < FallPnAInfoTot.size(); i++)
  {
    HJBAccWriter<<std::to_string(FallPnAInfoTot[i].HJBAccu)<<"\n";
  }
  HJBAccWriter.close();

  string HJBPreTrajFile = std::to_string(CaseNumber) + "_HJB_Pre.txt";
  const char *HJBPreTrajFile_Name = HJBPreTrajFile.c_str();
  std::ofstream HJBPreWriter;
  HJBPreWriter.open(HJBPreTrajFile_Name);
  for (int i = 0; i < FallPnAInfoTot.size(); i++)
  {
    HJBPreWriter<<std::to_string(FallPnAInfoTot[i].HJBAccuFall)<<"\n";
  }
  HJBPreWriter.close();

  string CPCEAccTrajFile = std::to_string(CaseNumber) + "_CPCE_Acc.txt";
  const char *CPCEAccTrajFile_Name = CPCEAccTrajFile.c_str();
  std::ofstream CPCEAccWriter;
  CPCEAccWriter.open(CPCEAccTrajFile_Name);
  for (int i = 0; i < FallPnAInfoTot.size(); i++)
  {
    CPCEAccWriter<<std::to_string(FallPnAInfoTot[i].CPCEAccu)<<"\n";
  }
  CPCEAccWriter.close();

  string CPCEPreTrajFile = std::to_string(CaseNumber) + "_CPCE_Pre.txt";
  const char *CPCEPreTrajFile_Name = CPCEPreTrajFile.c_str();
  std::ofstream CPCEPreWriter;
  CPCEPreWriter.open(CPCEPreTrajFile_Name);
  for (int i = 0; i < FallPnAInfoTot.size(); i++)
  {
    CPCEPreWriter<<std::to_string(FallPnAInfoTot[i].CPCEAccuFall)<<"\n";
  }
  CPCEPreWriter.close();


  string CPSPAccTrajFile = std::to_string(CaseNumber) + "_CPSP_Acc.txt";
  const char *CPSPAccTrajFile_Name = CPSPAccTrajFile.c_str();
  std::ofstream CPSPAccWriter;
  CPSPAccWriter.open(CPSPAccTrajFile_Name);
  for (int i = 0; i < FallPnAInfoTot.size(); i++)
  {
    CPSPAccWriter<<std::to_string(FallPnAInfoTot[i].CPSPAccu)<<"\n";
  }
  CPSPAccWriter.close();

  string CPSPPreTrajFile = std::to_string(CaseNumber) + "_CPSP_Pre.txt";
  const char *CPSPPreTrajFile_Name = CPSPPreTrajFile.c_str();
  std::ofstream CPSPPreWriter;
  CPSPPreWriter.open(CPSPPreTrajFile_Name);
  for (int i = 0; i < FallPnAInfoTot.size(); i++)
  {
    CPSPPreWriter<<std::to_string(FallPnAInfoTot[i].CPSPAccuFall)<<"\n";
  }
  CPSPPreWriter.close();

  string ZSCAccTrajFile = std::to_string(CaseNumber) + "_ZSC_Acc.txt";
  const char *ZSCAccTrajFile_Name = ZSCAccTrajFile.c_str();
  std::ofstream ZSCAccWriter;
  ZSCAccWriter.open(ZSCAccTrajFile_Name);
  for (int i = 0; i < FallPnAInfoTot.size(); i++)
  {
    ZSCAccWriter<<std::to_string(FallPnAInfoTot[i].ZSCAccu)<<"\n";
  }
  ZSCAccWriter.close();

  string ZSCPreTrajFile = std::to_string(CaseNumber) + "_ZSC_Pre.txt";
  const char *ZSCPreTrajFile_Name = ZSCPreTrajFile.c_str();
  std::ofstream ZSCPreWriter;
  ZSCPreWriter.open(ZSCPreTrajFile_Name);
  for (int i = 0; i < FallPnAInfoTot.size(); i++)
  {
    ZSCPreWriter<<std::to_string(FallPnAInfoTot[i].ZSCAccuFall)<<"\n";
  }
  ZSCPreWriter.close();

}

static std::vector<double> ObjTrajLoader(const int & CaseNumber, const int & ExpNumber, const string & FallDetector)
{
  string UserPath = "/home/shihao/Desktop/Stabilizability-Analysis-with-Polytopic-Viability-Kernel/ExpData";
  string CasePath, FallResName;
  switch (CaseNumber)
  {
    case 1:
    {
      CasePath = "/Case 1/";
      FallResName = "GndTruthCase1.txt";
    }
    break;
    case 2:
    {
      CasePath = "/Case 2/";
      FallResName = "GndTruthCase2.txt";
    }
    break;
    case 3:
    {
      CasePath = "/Case 3/";
      FallResName = "GndTruthCase3.txt";
    }
    break;
    case 4:
    {
      CasePath = "/Case 4/";
      FallResName = "GndTruthCase4.txt";
    }
    break;
    case 5:
    {
      CasePath = "/Case 5/";
      FallResName = "GndTruthCase5.txt";
    }
    break;
    case 6:
    {
      CasePath = "/Case 6/";
      FallResName = "GndTruthCase6.txt";
    }
    break;
    default:
    {
      std::printf("Case Number does not exist!\n");
    }
    break;
  }

  // This function is used to load in computed trajectories.
  const string ObjTrajFile = FallDetector + "Traj" + std::to_string(ExpNumber) + ".txt";
  const char *ObjTrajFile_Name = ObjTrajFile.c_str();

  string str_line;
  string ObjTrajFilePath = UserPath + CasePath + ObjTrajFile_Name;
  ifstream ObjInfofile (ObjTrajFilePath);
  std::vector<double> ObjTraj;
  if (ObjInfofile.is_open())
  {
    while (getline(ObjInfofile, str_line) )
    {
      ObjTraj.push_back(stod(str_line));
    }
    ObjInfofile.close();
  }
  else
  {
    std::cout<<"Wrong! Obj File cannot be found!"<<endl;
  }
  return ObjTraj;
}

static void ROCWriter(const std::vector<double> & TPRTraj, const std::vector<double> & FPRTraj, const int & CaseNumber, const string FallDetector)
{
  // This function is used to generate the ROC curve
  string TPRTrajFile = FallDetector + "_" + std::to_string(CaseNumber) + "_TPR.txt";
  const char *TPRTrajFile_Name = TPRTrajFile.c_str();
  std::ofstream TPRWriter;
  TPRWriter.open(TPRTrajFile_Name);
  for (int i = 0; i < TPRTraj.size(); i++)
  {
    TPRWriter<<std::to_string(TPRTraj[i])<<"\n";
  }
  TPRWriter.close();

  string FPRTrajFile = FallDetector + "_" + std::to_string(CaseNumber) + "_FPR.txt";
  const char *FPRTrajFile_Name = FPRTrajFile.c_str();
  std::ofstream FPRWriter;
  FPRWriter.open(FPRTrajFile_Name);
  for (int i = 0; i < TPRTraj.size(); i++)
  {
    FPRWriter<<std::to_string(FPRTraj[i])<<"\n";
  }
  FPRWriter.close();
}

static std::vector<double> linspace(double start_in, double end_in, int num_in)
{
  std::vector<double> linspaced;
  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);
  if (num == 0) { return linspaced; }
  if (num == 1)
    {
      linspaced.push_back(start);
      return linspaced;
    }
  double delta = (end - start) / (num - 1);
  for(int i=0; i < num-1; ++i)
    {
      linspaced.push_back(start + delta * i);
    }
  linspaced.push_back(end); // I want to ensure that start and end
                            // are exactly the same as the input
  return linspaced;
}

static std::vector<double> getVertexIndices(std::string const& pointLine)
{
  std::istringstream iss(pointLine);

  return std::vector<double>{
    std::istream_iterator<double>(iss),
    std::istream_iterator<double>()
  };
}

int qTrajNCOMVelTrajLoader(const int & CaseNumber, const int& FileIndex, std::vector<Config> & qTraj, std::vector<double> & COMVelx, std::vector<double> & COMVely, std::vector<double> & COMVelz)
{
  // This function is used to load in qTraj and qdotTraj.
  // Also the fall result will be loaded as well.
  // There are 6 cases used in our project.

  string UserPath = "/home/shihao/Desktop/Stabilizability-Analysis-with-Polytopic-Viability-Kernel/build/ExpData";
  string CasePath, FallResName;
  switch (CaseNumber)
  {
    case 1:
    {
      CasePath = "/Case 1/";
      FallResName = "GndTruthCase1.txt";
    }
    break;
    case 2:
    {
      CasePath = "/Case 2/";
      FallResName = "GndTruthCase2.txt";
    }
    break;
    case 3:
    {
      CasePath = "/Case 3/";
      FallResName = "GndTruthCase3.txt";
    }
    break;
    case 4:
    {
      CasePath = "/Case 4/";
      FallResName = "GndTruthCase4.txt";
    }
    break;
    case 5:
    {
      CasePath = "/Case 5/";
      FallResName = "GndTruthCase5.txt";
    }
    break;
    case 6:
    {
      CasePath = "/Case 6/";
      FallResName = "GndTruthCase6.txt";
    }
    break;
    default:
    {
      std::printf("Case Number does not exist!\n");
    }
    break;
  }
  const string qTrajFile = "stateTraj" + std::to_string(FileIndex) + ".path";
  const char *qTrajFile_Name = qTrajFile.c_str();

  string str_line;
  string ConfigFilePath = UserPath + CasePath + qTrajFile_Name;
  ifstream ConfigInfofile (ConfigFilePath);
  if (ConfigInfofile.is_open())
  {
    while (getline(ConfigInfofile, str_line) )
    {
      std::vector<double> RobotConfigTime = getVertexIndices(str_line);
      std::vector<double> RobotConfig(RobotConfigTime.begin()+2, RobotConfigTime.end());
      // VectorPrintResult(RobotConfig);
      Config RobotConfig_i(RobotConfig);
      qTraj.push_back(RobotConfig_i);
    }
    ConfigInfofile.close();
  }
  else
  {
    std::cout<<"Wrong! State Traj cannot be found!"<<endl;
  }

  // The last file to be loaded is the Centroidal velocity.
  string COMVelxFile = "COMVelxTraj" + std::to_string(FileIndex) + ".txt";
  string COMVelxFilePath = UserPath + CasePath + COMVelxFile;
  ifstream COMVelxfile(COMVelxFilePath);
  if (COMVelxfile.is_open())
  {
    while (getline(COMVelxfile, str_line) )
    {
      COMVelx.push_back(std::stoi(str_line));
    }
    COMVelxfile.close();
  }
  else
  {
    std::cout<<"Wrong! COMVel x txt cannot be found!"<<endl;
  }

  string COMVelyFile = "COMVelyTraj" + std::to_string(FileIndex) + ".txt";
  string COMVelyFilePath = UserPath + CasePath + COMVelyFile;
  ifstream COMVelyfile(COMVelyFilePath);
  if (COMVelyfile.is_open())
  {
    while (getline(COMVelyfile, str_line) )
    {
      COMVely.push_back(std::stoi(str_line));
    }
    COMVelyfile.close();
  }
  else
  {
    std::cout<<"Wrong! COMVel x txt cannot be found!"<<endl;
  }

  string COMVelzFile = "COMVelzTraj" + std::to_string(FileIndex) + ".txt";
  string COMVelzFilePath = UserPath + CasePath + COMVelzFile;
  ifstream COMVelzfile(COMVelzFilePath);
  if (COMVelzfile.is_open())
  {
    while (getline(COMVelzfile, str_line) )
    {
      COMVelz.push_back(std::stoi(str_line));
    }
    COMVelzfile.close();
  }
  else
  {
    std::cout<<"Wrong! COMVel x txt cannot be found!"<<endl;
  }

  string FallResPath = UserPath + CasePath + FallResName;
  std::vector<int> FallStatus;
  ifstream FallInfofile(FallResPath);
  if (FallInfofile.is_open())
  {
    while (getline(FallInfofile, str_line) )
    {
      FallStatus.push_back(std::stoi(str_line));
    }
    FallInfofile.close();
  }
  else
  {
    std::cout<<"Wrong! FallStatus txt cannot be found!"<<endl;
  }
  return FallStatus[FileIndex-1];
}

static int qTrajNqdotTrajLoader(const int & CaseNumber, const int& FileIndex, std::vector<Config> & qTraj, std::vector<Config> & qdotTraj, std::vector<double> & COMVelx, std::vector<double> & COMVely, std::vector<double> & COMVelz)
{
  // This function is used to load in qTraj and qdotTraj.
  // Also the fall result will be loaded as well.
  // There are 6 cases used in our project.

  string UserPath = "/home/shihao/Desktop/Stabilizability-Analysis-with-Polytopic-Viability-Kernel/build/ExpData";
  string CasePath, FallResName;
  switch (CaseNumber)
  {
    case 1:
    {
      CasePath = "/Case 1/";
      FallResName = "GndTruthCase1.txt";
    }
    break;
    case 2:
    {
      CasePath = "/Case 2/";
      FallResName = "GndTruthCase2.txt";
    }
    break;
    case 3:
    {
      CasePath = "/Case 3/";
      FallResName = "GndTruthCase3.txt";
    }
    break;
    case 4:
    {
      CasePath = "/Case 4/";
      FallResName = "GndTruthCase4.txt";
    }
    break;
    case 5:
    {
      CasePath = "/Case 5/";
      FallResName = "GndTruthCase5.txt";
    }
    break;
    case 6:
    {
      CasePath = "/Case 6/";
      FallResName = "GndTruthCase6.txt";
    }
    break;
    default:
    {
      std::printf("Case Number does not exist!\n");
    }
    break;
  }
  const string qTrajFile = "qTrajAct" + std::to_string(FileIndex) + ".txt";
  const char *qTrajFile_Name = qTrajFile.c_str();
  const string qdotTrajFile = "qdotTrajAct" + std::to_string(FileIndex) + ".txt";
  const char *qdotTrajFile_Name = qdotTrajFile.c_str();

  string str_line;
  string ConfigFilePath = UserPath + CasePath + qTrajFile_Name;
  ifstream ConfigInfofile (ConfigFilePath);
  if (ConfigInfofile.is_open())
  {
    while (getline(ConfigInfofile, str_line) )
    {
      std::vector<double> RobotConfig = getVertexIndices(str_line);
      // VectorPrintResult(RobotConfig);
      Config RobotConfig_i(RobotConfig);
      qTraj.push_back(RobotConfig_i);
    }
    ConfigInfofile.close();
  }
  else
  {
    std::cout<<"Wrong! config txt cannot be found!"<<endl;
  }

  string VelocityFilePath = UserPath + CasePath + qdotTrajFile_Name;
  ifstream VelocityInfofile (VelocityFilePath);
  if (VelocityInfofile.is_open())
  {
    while (getline (VelocityInfofile, str_line) )
    {
      // std::cout << str_line << "\n";
      std::vector<double> RobotVelocity = getVertexIndices(str_line);
      // VectorPrintResult(RobotVelocity);
      Config RobotVelocity_i(RobotVelocity);
      qdotTraj.push_back(RobotVelocity_i);
    }
    VelocityInfofile.close();
  }
  else
  {
    std::cout<<"Wrong! velocity txt cannot be found!"<<endl;
  }

  // The last file to be loaded is FallRes.
  string FallResPath = UserPath + CasePath + FallResName;
  std::vector<int> FallStatus;
  ifstream FallInfofile(FallResPath);
  if (FallInfofile.is_open())
  {
    while (getline(FallInfofile, str_line) )
    {
      FallStatus.push_back(std::stoi(str_line));
    }
    FallInfofile.close();
  }
  else
  {
    std::cout<<"Wrong! FallStatus txt cannot be found!"<<endl;
  }

  // The last file to be loaded is the Centroidal velocity.
  string COMVelxFile = "COMVelxTraj" + std::to_string(FileIndex) + ".txt";
  string COMVelxFilePath = UserPath + CasePath + COMVelxFile;
  ifstream COMVelxfile(COMVelxFilePath);
  if (COMVelxfile.is_open())
  {
    while (getline(COMVelxfile, str_line) )
    {
      COMVelx.push_back(std::stoi(str_line));
    }
    COMVelxfile.close();
  }
  else
  {
    std::cout<<"Wrong! COMVel x txt cannot be found!"<<endl;
  }

  string COMVelyFile = "COMVelyTraj" + std::to_string(FileIndex) + ".txt";
  string COMVelyFilePath = UserPath + CasePath + COMVelyFile;
  ifstream COMVelyfile(COMVelyFilePath);
  if (COMVelyfile.is_open())
  {
    while (getline(COMVelyfile, str_line) )
    {
      COMVely.push_back(std::stoi(str_line));
    }
    COMVelyfile.close();
  }
  else
  {
    std::cout<<"Wrong! COMVel x txt cannot be found!"<<endl;
  }

  string COMVelzFile = "COMVelzTraj" + std::to_string(FileIndex) + ".txt";
  string COMVelzFilePath = UserPath + CasePath + COMVelzFile;
  ifstream COMVelzfile(COMVelzFilePath);
  if (COMVelzfile.is_open())
  {
    while (getline(COMVelzfile, str_line) )
    {
      COMVelz.push_back(std::stoi(str_line));
    }
    COMVelzfile.close();
  }
  else
  {
    std::cout<<"Wrong! COMVel x txt cannot be found!"<<endl;
  }
  return FallStatus[FileIndex-1];
}

static FallPnAInfo DataAnalysisInner(const int & CaseNumber, const int & CutOffIndex)
{
  int FileTot = 350;

  int RBAccuFallNumber = 0;
  int HJBAccuFallNumber = 0;
  int CPCEAccuFallNumber = 0;
  int CPSPAccuFallNumber = 0;
  int ZSCAccuFallNumber = 0;

  int RBAccuNumber = 0;
  int HJBAccuNumber = 0;
  int CPCEAccuNumber = 0;
  int CPSPAccuNumber = 0;
  int ZSCAccuNumber = 0;

  int TotalFall = 0;

  for (int FileIndex = 1; FileIndex < FileTot; FileIndex++)
  {
    // std::printf("File Index: %d\n", FileIndex);
    std::vector<double> RBTraj =    ObjTrajLoader(CaseNumber, FileIndex, "RB");
    std::vector<double> HJBTraj =   ObjTrajLoader(CaseNumber, FileIndex, "HJB");
    std::vector<double> CPCETraj =  ObjTrajLoader(CaseNumber, FileIndex, "CPCE");
    std::vector<double> CPSPTraj =  ObjTrajLoader(CaseNumber, FileIndex, "CPSP");
    std::vector<double> ZSCTraj =   ObjTrajLoader(CaseNumber, FileIndex, "ZSC");

    std::vector <Config> qTraj, qdotTraj;
    std::vector <double> COMVelx, COMVely, COMVelz;
    int FallStatus = qTrajNqdotTrajLoader(CaseNumber, FileIndex, qTraj, qdotTraj, COMVelx, COMVely, COMVelz);
    TotalFall+= FallStatus;

    int RBObj =   FallStatusFinder(RBTraj, CutOffIndex);
    int HJBObj =  FallStatusFinder(HJBTraj, CutOffIndex);
    int CPCEObj = FallStatusFinder(CPCETraj,  CutOffIndex);
    int CPSPObj = FallStatusFinder(CPSPTraj, CutOffIndex);
    int ZSCObj =  FallStatusFinder(ZSCTraj, CutOffIndex);

    if(RBObj == FallStatus)
    {
      RBAccuNumber++;
      if(FallStatus == 1)
      {
        RBAccuFallNumber++;
      }
    }

    if(HJBObj == FallStatus)
    {
      HJBAccuNumber++;
      if(FallStatus == 1)
      {
        HJBAccuFallNumber++;
      }
    }

    if(CPCEObj == FallStatus)
    {
      CPCEAccuNumber++;
      if(FallStatus == 1)
      {
        CPCEAccuFallNumber++;
      }
    }

    if(CPSPObj == FallStatus)
    {
      CPSPAccuNumber++;
      if(FallStatus == 1)
      {
        CPSPAccuFallNumber++;
      }
    }

    if(ZSCObj == FallStatus)
    {
      ZSCAccuNumber++;
      if(FallStatus == 1)
      {
        ZSCAccuFallNumber++;
      }
    }
  }

  double RBAccu = 1.0 * RBAccuNumber/(1.0 * FileTot);
  double RBAccuFall = 1.0 * RBAccuFallNumber/(1.0 * TotalFall);

  double HJBAccu = 1.0 * HJBAccuNumber/(1.0 * FileTot);
  double HJBAccuFall = 1.0 * HJBAccuFallNumber/(1.0 * TotalFall);

  double CPCEAccu = 1.0 * CPCEAccuNumber/(1.0 * FileTot);
  double CPCEAccuFall = 1.0 * CPCEAccuFallNumber/(1.0 * TotalFall);

  double CPSPAccu = 1.0 * CPSPAccuNumber/(1.0 * FileTot);
  double CPSPAccuFall = 1.0 * CPSPAccuFallNumber/(1.0 * TotalFall);

  double ZSCAccu = 1.0 * ZSCAccuNumber/(1.0 * FileTot);
  double ZSCAccuFall = 1.0 * ZSCAccuFallNumber/(1.0 * TotalFall);

  FallPnAInfo FallPnAInfoObj(RBAccu, RBAccuFall, HJBAccu, HJBAccuFall, CPCEAccu, CPCEAccuFall, CPSPAccu, CPSPAccuFall, ZSCAccu, ZSCAccuFall);
  return FallPnAInfoObj;
}


static void ObjTrajWriter(const std::vector<double> & Traj, const int & ExpIndex, const string & FallDetector)
{
  // This function is used to generate the ROC curve
  string TPRTrajFile = FallDetector + "Traj" + std::to_string(ExpIndex) + ".txt";
  const char *TPRTrajFile_Name = TPRTrajFile.c_str();
  std::ofstream TPRWriter;
  TPRWriter.open(TPRTrajFile_Name);
  for (int i = 0; i < Traj.size(); i++)
  {
    TPRWriter<<std::to_string(Traj[i])<<"\n";
  }
  TPRWriter.close();
}


static std::vector<int> FailureMethodEvaluation(const int & ExpIndex, Robot& SimRobot, ViabilityKernelInfo& VKObj, std::vector<LinkInfo> & RobotLinkInfo, std::vector<ContactStatusInfo> & RobotContactInfo, SignedDistanceFieldInfo & SDFInfo, const std::vector<Config> & qTraj, const std::vector<Config> & qdotTraj, const std::vector<double> & COMVelx, const std::vector<double> & COMVely, const std::vector<double> & COMVelz , const std::vector<double> & Margins)
{
  int EdgeNumber = 4;
  double mu = 0.5;
  double mass = 56.5;

  // This function takes in the given trajectory of q and qdot and output for each method.
  double dt = 0.025;        // This should be set in advance according to simulation.
  Config qPre = qTraj[0];
  Config qdotPre = qdotTraj[0];
  Vector3 COMPosPre(0.0, 0.0, 0.0);
  Vector3 COMVelPre(0.0, 0.0, 0.0);
  Vector3 COMPosNow(0.0, 0.0, 0.0);
  Vector3 COMVelNow(0.0, 0.0, 0.0);
  SimRobot.UpdateConfig(qPre);
  SimRobot.dq = qdotPre;
  CentroidalState(SimRobot, COMPosPre, COMVelPre);
  double PVKRBMargin = Margins[0];
  double PVKCPMargin = Margins[1];
  double PVKHJBMargin = Margins[2];
  double OEMargin = Margins[3];
  double CPMargin = Margins[4];
  double ZMPMargin = Margins[5];
  double ZSCMargin = Margins[6];
  double CPFSPMargin = Margins[7];

  std::vector<double> PVKRBTraj(qTraj.size()-1);
  std::vector<double> PVKCPTraj(qTraj.size()-1);
  std::vector<double> PVKHJBTraj(qTraj.size()-1);
  std::vector<double> OETraj(qTraj.size()-1);
  std::vector<double> CPTraj(qTraj.size()-1);
  std::vector<double> ZMPTraj(qTraj.size()-1);
  std::vector<double> ZSCTraj(qTraj.size()-1);
  std::vector<double> CPFSPTraj(qTraj.size()-1);

  int StepIndex = 0;
  for (int i = 1; i < qTraj.size(); i++)        // For the sake of Centroidal Acceleration for ZMP
  {
    Config qNow = qTraj[i];
    Config qdotNow = qdotTraj[i];
    SimRobot.UpdateConfig(qNow);
    SimRobot.dq = qdotNow;
    // VectorPrintResult(qNow);
    // VectorPrintResult(qdotNow);
    CentroidalState(SimRobot, COMPosNow, COMVelNow);
    Vector3 COMAcc(0.0, 0.0, 0.0);
    COMAcc.x = (COMVelx[i] - COMVelx[i-1])/dt;
    COMAcc.y = (COMVely[i] - COMVely[i-1])/dt;
    COMAcc.z = (COMVelz[i] - COMVelz[i-1])/dt;

    Matrix pCOMpq;
    SimRobot.GetCOMJacobian(pCOMpq);
    std::vector<Vector3>  ActContactPositions, ActVelocities;        // A vector of Vector3 points
    std::vector<Matrix>   ActJacobians;       // A vector of Jacobian matrices
    std::vector<int> ActStatus;
    ActContactNJacobian(SimRobot, RobotLinkInfo, RobotContactInfo, ActContactPositions, ActVelocities, ActJacobians, ActStatus, SDFInfo);

    double mu_t = mu * ZSCMargin;
    std::vector<Vector3> ConeAllUnit, ConeUnits;
    ConeUnitGenerator(ActContactPositions, SDFInfo, ConeAllUnit, ConeUnits, EdgeNumber, mu_t);

    // This is the value for PVK-HJB
    std::vector<double> PIPObj;
    std::vector<Vector3> COMDesVector;
    std::vector<int> StatusVector;
    double HJBObjective;
    std::vector<PIPInfo> PIPTotal = PIPGeneratorAnalysis(ActContactPositions, ActVelocities, ActStatus, COMPosNow, COMVelNow, VKObj, PIPObj, HJBObjective, PVKHJBMargin);
    PVKHJBTraj[StepIndex] = HJBObjective;

    // This is the value for PVK-RB
    double PVKRBObjective = RBGeneratorAnalysis(PIPTotal, PVKRBMargin);
    PVKRBTraj[StepIndex] = PVKRBObjective;

    // This is the value for PVK-CP
    double PVKCPObjective = CPCEGeneratorAnalysis(PIPTotal, PVKCPMargin);
    PVKCPTraj[StepIndex] = PVKCPObjective;

    // Projection to ground
    std::vector<Vector3> ProjActContactPositions;
    ProjActContactPositions.reserve(ActContactPositions.size());

    double LowestHeight = 100.0;
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

    // Support Polygon should be generated
    std::vector<PIPInfo> PIPSPTotal = PIPGeneratorAnalysis(ProjActContactPositions, ActVelocities, ActStatus, COMPosNow, COMVelNow, VKObj, PIPObj, HJBObjective, PVKHJBMargin);

    // Orbital Energy which is a 2D version of PVK-RB
    double OEObjective = RBGeneratorAnalysis(PIPSPTotal, OEMargin);
    OETraj[StepIndex] = OEObjective;

    // Capture Point which is a 2D versino of PVK-CP
    double CPObjective = CPCEGeneratorAnalysis(PIPSPTotal, CPMargin);
    CPTraj[StepIndex] = CPObjective;

    // ZMP
    double ZMPObjective = ZMPGeneratorAnalysis(PIPSPTotal, COMPosNow, COMAcc, ZMPMargin);
    ZMPTraj[StepIndex] = ZMPObjective;
    //
    // // ZSC
    // double ZSCObjective = ZeroStepCapturabilityGenerator(ActContactPositions, ConeAllUnit, EdgeNumber, COMPosNow, COMVelNow);
    // ZSCTraj[StepIndex] = ZSCObjective;

    // // CP with FSP
    // double CPSPObjective = CPSPGenerator(ActContactPositions, COMPosNow, COMVelNow, mass, ConeUnits, EdgeNumber);
    // CPFSPTraj[StepIndex] = CPSPObjective;

    StepIndex = StepIndex + 1;
  }

  int PVKRBRes = FallStatusFinder(PVKRBTraj, StepIndex);
  int PVKCPRes = FallStatusFinder(PVKCPTraj, StepIndex);
  int PVKHJBRes = FallStatusFinder(PVKHJBTraj, StepIndex);
  int OERes = FallStatusFinder(OETraj, StepIndex);
  int CPRes = FallStatusFinder(CPTraj, StepIndex);
  int ZMPRes = FallStatusFinder(ZMPTraj, StepIndex);
  int ZSCRes = FallStatusFinder(ZSCTraj, StepIndex);
  int CPFSPRes = FallStatusFinder(CPFSPTraj, StepIndex);

  // All other trajectories have to be saved together.

  ObjTrajWriter(OETraj, ExpIndex, "OE");
  ObjTrajWriter(CPTraj, ExpIndex, "CP");
  ObjTrajWriter(ZMPTraj, ExpIndex, "ZMP");

  std::vector<int> Status = {PVKRBRes, PVKCPRes, PVKHJBRes, OERes, CPRes, ZMPRes, ZSCRes, CPFSPRes};
  return Status;
}

static void StatusClassification(const int & GndTruth, const int & PreRes, int & TP, int & FP, int & TN, int & FN)
{
  switch (GndTruth)
  {
    case 1:
    {
      switch (PreRes)
      {
        case 1:
        {
          TP+=1;
        }
        break;
        case 0:
        {
          FN+=1;
        }
        break;
        default:
        {
        }
        break;
      }
    }
    break;
    case 0:
    {
      switch (PreRes)
      {
        case 1:
        {
          FP+=1;
        }
        break;
        case 0:
        {
          TN+=1;
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
    }
    break;
  }
  return;

}

static std::pair<double, double> TPRnFPR(const int & TP, const int & FP, const int & TN, const int & FN)
{
  double TPR = (1.0 * TP)/(1.0 * TP + 1.0 * FN);
  double FPR = (1.0 * FP)/(1.0 * FP + 1.0 * TN);
  return std::make_pair (TPR,FPR);
}

std::vector<pair <double, double>> ROCCurveInner(const int & CaseNumber,  Robot& SimRobot, ViabilityKernelInfo& VKObj, std::vector<LinkInfo> & RobotLinkInfo, std::vector<ContactStatusInfo> & RobotContactInfo, SignedDistanceFieldInfo & SDFInfo, std::vector<double> & Margins)
{
  // This function takes the responsibility of core computation.

  int FileIndexTotal = 100;
  int PVKRBTP = 0;          int PVKRBFP = 0;          int PVKRBTN = 0;          int PVKRBFN = 0;
  int PVKCPTP = 0;          int PVKCPFP = 0;          int PVKCPTN = 0;          int PVKCPFN = 0;
  int PVKHJBTP = 0;         int PVKHJBFP = 0;         int PVKHJBTN = 0;         int PVKHJBFN = 0;
  int OETP = 0;             int OEFP = 0;             int OETN = 0;             int OEFN = 0;
  int CPTP = 0;             int CPFP = 0;             int CPTN = 0;             int CPFN = 0;
  int ZMPTP = 0;            int ZMPFP = 0;            int ZMPTN = 0;            int ZMPFN = 0;
  int ZSCTP = 0;            int ZSCFP = 0;            int ZSCTN = 0;            int ZSCFN = 0;
  int CPFSPTP = 0;          int CPFSPFP = 0;          int CPFSPTN = 0;          int CPFSPFN = 0;

  for (int FileIndex = 1; FileIndex < FileIndexTotal; FileIndex++)
  {
    // std::printf("File Index: %d\n", FileIndex);
    // clock_t begin = clock();
    std::vector<Config> qTraj, qdotTraj;
    std::vector<double> COMVelx, COMVely, COMVelz;
    int FallStatus = qTrajNqdotTrajLoader(CaseNumber, FileIndex, qTraj, qdotTraj, COMVelx, COMVely, COMVelz);
    std::vector<int> FallPredictStatus = FailureMethodEvaluation(FileIndex, SimRobot, VKObj, RobotLinkInfo, RobotContactInfo, SDFInfo, qTraj, qdotTraj, COMVelx, COMVely, COMVelz, Margins);

    int PVKRB = FallPredictStatus[0];
    int PVKCP = FallPredictStatus[1];
    int PVKHJB = FallPredictStatus[2];
    int OE = FallPredictStatus[3];
    int CP = FallPredictStatus[4];
    int ZMP = FallPredictStatus[5];
    int ZSC = FallPredictStatus[6];
    int CPFSP = FallPredictStatus[7];

    StatusClassification(FallStatus, PVKRB,   PVKRBTP,        PVKRBFP,          PVKRBTN,            PVKRBFN);
    StatusClassification(FallStatus, PVKCP,   PVKCPTP,        PVKCPFP,          PVKCPTN,            PVKCPFN);
    StatusClassification(FallStatus, PVKHJB,  PVKHJBTP,       PVKHJBFP,         PVKHJBTN,           PVKHJBFN);
    StatusClassification(FallStatus, OE,      OETP,           OEFP,             OETN,               OEFN);
    StatusClassification(FallStatus, CP,      CPTP,           CPFP,             CPTN,               CPFN);
    StatusClassification(FallStatus, ZMP,     ZMPTP,          ZMPFP,            ZMPTN,              ZMPFN);
    StatusClassification(FallStatus, ZSC,     ZSCTP,          ZSCFP,            ZSCTN,              ZSCFN);
    StatusClassification(FallStatus, CPFSP,   CPFSPTP,        CPFSPFP,          CPFSPTN,            CPFSPFN);

    // clock_t end = clock();
    // double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    // std::printf("Time: %f\n", elapsed_secs);

  }
  std::pair<double, double> PVKRBRate = TPRnFPR(PVKRBTP,        PVKRBFP,          PVKRBTN,            PVKRBFN);
  std::pair<double, double> PVKCPRate = TPRnFPR(PVKCPTP,        PVKCPFP,          PVKCPTN,            PVKCPFN);
  std::pair<double, double> PVKHJBRate = TPRnFPR(PVKHJBTP,       PVKHJBFP,         PVKHJBTN,           PVKHJBFN);
  std::pair<double, double> OERate = TPRnFPR(OETP,           OEFP,             OETN,               OEFN);
  std::pair<double, double> CPRate = TPRnFPR(CPTP,           CPFP,             CPTN,               CPFN);
  std::pair<double, double> ZMPRate = TPRnFPR(ZMPTP,          ZMPFP,            ZMPTN,              ZMPFN);
  std::pair<double, double> ZSCRate = TPRnFPR(ZSCTP,          ZSCFP,            ZSCTN,              ZSCFN);
  std::pair<double, double> CPFSPRate = TPRnFPR(CPFSPTP,        CPFSPFP,          CPFSPTN,            CPFSPFN);

  std::vector<std::pair<double, double>> ROCRates = {PVKRBRate, PVKCPRate ,PVKHJBRate, OERate, CPRate, ZMPRate, ZSCRate, CPFSPRate};
  return ROCRates;
}

void ROCCurveGenerator(const int & CaseNumber, Robot& SimRobot, ViabilityKernelInfo& VKObj, std::vector<LinkInfo> & RobotLinkInfo, std::vector<ContactStatusInfo> & RobotContactInfo, SignedDistanceFieldInfo & SDFInfo)
{
  // This function is used to generat the ROC curve for experiment
  // const int CaseNumber = 1;
  const int MarginGrid = 101;

  std::vector<double> PVKRBMargin = linspace(0.0, 0.2, MarginGrid);
  std::vector<double> PVKCPMargin = linspace(0.0, 0.2, MarginGrid);
  std::vector<double> PVKHJBMargin = linspace(0.0, -0.2, MarginGrid);
  std::vector<double> OEMargin = linspace(0.0, 0.2, MarginGrid);
  std::vector<double> CPMargin = linspace(0.0, 0.2, MarginGrid);
  std::vector<double> ZMPMargin = linspace(0.0, 0.2, MarginGrid);
  std::vector<double> ZSCMargin = linspace(1.0, 0.0, MarginGrid);
  std::vector<double> CPFSPMargin = linspace(1.0, 0.0, MarginGrid);

  std::vector<double> PVKRBTPRTraj,   PVKRBFPRTraj;
  std::vector<double> PVKCPTPRTraj,   PVKCPFPRTraj;
  std::vector<double> PVKHJBTPRTraj,  PVKHJBFPRTraj;
  std::vector<double> OETPRTraj,      OEFPRTraj;
  std::vector<double> CPTPRTraj,      CPFPRTraj;
  std::vector<double> ZMPTPRTraj,     ZMPFPRTraj;
  std::vector<double> ZSCTPRTraj,     ZSCFPRTraj;
  std::vector<double> CPFSPTPRTraj,   CPFSPFPRTraj;

  for (int i = 0; i < MarginGrid; i++)
  {
    std::vector<double> Margins={ PVKRBMargin[i], PVKCPMargin[i], PVKHJBMargin[i], OEMargin[i], CPMargin[i], ZMPMargin[i], ZSCMargin[i], CPFSPMargin[i]};
    std::vector<pair <double, double>> ROCCurve_i = ROCCurveInner(CaseNumber, SimRobot, VKObj, RobotLinkInfo, RobotContactInfo, SDFInfo, Margins);
    std::pair <double, double> PVKRBRates = ROCCurve_i[0];
    std::pair <double, double> PVKCPRates = ROCCurve_i[1];
    std::pair <double, double> PVKHJRates = ROCCurve_i[2];
    std::pair <double, double> OERates = ROCCurve_i[3];
    std::pair <double, double> CPRates = ROCCurve_i[4];
    std::pair <double, double> ZMPRates = ROCCurve_i[5];
    std::pair <double, double> ZSCRates = ROCCurve_i[6];
    std::pair <double, double> CPFSPRates = ROCCurve_i[7];

    printf("Margin Grid: %d\n", i);
    printf("PVKRB: TPR %f FPR %f\n", PVKRBRates.first, PVKRBRates.second);
    printf("PVKCP: TPR %f FPR %f\n", PVKCPRates.first, PVKCPRates.second);
    printf("PVKHJB: TPR %f FPR %f\n", PVKHJRates.first, PVKHJRates.second);
    printf("OE: TPR %f FPR %f\n", OERates.first, OERates.second);
    printf("CP: TPR %f FPR %f\n", CPRates.first, CPRates.second);
    printf("ZMP: TPR %f FPR %f\n", ZMPRates.first, ZMPRates.second);
    printf("ZSC: TPR %f FPR %f\n", ZSCRates.first, ZSCRates.second);
    printf("CPFSP: TPR %f FPR %f\n", CPFSPRates.first, CPFSPRates.second);

    PVKRBTPRTraj.push_back(PVKRBRates.first);       PVKRBFPRTraj.push_back(PVKRBRates.second);
    PVKCPTPRTraj.push_back(PVKCPRates.first);       PVKCPFPRTraj.push_back(PVKCPRates.second);
    PVKHJBTPRTraj.push_back(PVKHJRates.first);      PVKHJBFPRTraj.push_back(PVKHJRates.second);
    OETPRTraj.push_back(OERates.first);             OEFPRTraj.push_back(OERates.second);
    CPTPRTraj.push_back(CPRates.first);             CPFPRTraj.push_back(CPRates.second);
    ZMPTPRTraj.push_back(ZMPRates.first);           ZMPFPRTraj.push_back(ZMPRates.second);
    ZSCTPRTraj.push_back(ZSCRates.first);           ZSCFPRTraj.push_back(ZSCRates.second);
    CPFSPTPRTraj.push_back(CPFSPRates.first);       CPFSPFPRTraj.push_back(CPFSPRates.second);
  }
  ROCWriter(PVKRBTPRTraj, PVKRBFPRTraj, CaseNumber, "PVKRB");
  ROCWriter(PVKCPTPRTraj, PVKCPFPRTraj, CaseNumber, "PVKCP");
  ROCWriter(PVKHJBTPRTraj, PVKHJBFPRTraj, CaseNumber, "PVKHJB");
  ROCWriter(OETPRTraj, OEFPRTraj, CaseNumber, "OE");
  ROCWriter(CPTPRTraj, CPFPRTraj, CaseNumber, "CP");
  ROCWriter(ZMPTPRTraj, ZMPFPRTraj, CaseNumber, "ZMP");
  ROCWriter(ZSCTPRTraj, ZSCFPRTraj, CaseNumber, "ZSC");
  ROCWriter(CPFSPTPRTraj, CPFSPFPRTraj, CaseNumber, "CPFSP");
  return;
}

void DataAnalysis(int & CaseNumber)
{
  // This function is use to generate data analysis for experimentation trajectories.
  int TotalIndex = 100;
  std::vector<FallPnAInfo> FallPnAInfoTot;
  FallPnAInfoTot.reserve(TotalIndex);
  for (int CutOffIndex = 0; CutOffIndex < TotalIndex; CutOffIndex++)
  {
    std::printf("CutOffIndex: %d\n", CutOffIndex);
    FallPnAInfo FallPnAInfoObj = DataAnalysisInner(CaseNumber, CutOffIndex);
    FallPnAInfoTot.push_back(FallPnAInfoObj);
  }

  //   duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  FallPnAInfoTotWriter(FallPnAInfoTot, CaseNumber);


  return;
}
