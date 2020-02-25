// This function is used to calculate certain robot utility functions
#include "RobotInfo.h"
#include "CommonHeader.h"
#include <KrisLibrary/robotics/Inertia.h>
#include <random>
#include <limits>
using namespace std;

void SimRobotToRobotState(const Robot &_SimRobot, std::vector<double>& _Config, std::vector<double>& _Velocity)
{
  // Here, the robot's configuration and velocity must have already been initialized
  for (int i = 0; i < _SimRobot.q.size(); i++)
  {
    _Config[i] = _SimRobot.q[i];
    _Velocity[i] = _SimRobot.dq[i];
  }
  return;
}

vector<double> SimRobotToRobotState(const Robot &_SimRobot)
{
  // Here, the robot's configuration and velocity must have already been initialized
  vector<double> RobotState(2 * _SimRobot.q.size());
  for (int i = 0; i < _SimRobot.q.size(); i++)
  {
    RobotState[i] = _SimRobot.q[i];
    RobotState[i + _SimRobot.q.size()] = _SimRobot.dq[i];
  }
  return RobotState;
}
void RobotStateToSimRobot(Robot & SimRobot, const std::vector<double> & RobotState)
{
  // This function does not contain the update of dB/dq

  std::vector<double> RobotConfig(RobotState.begin(), RobotState.begin() + RobotState.size()/2);
  std::vector<double> RobotVelocity(RobotState.begin()+ RobotState.size()/2, RobotState.end());

  RobotStateToSimRobot(SimRobot, RobotConfig, RobotVelocity);
}

void RobotStateToSimRobot(Robot & SimRobot, const std::vector<double> &RobotConfig, const std::vector<double> & RobotVelocity)
{
  Config RobotConfigNew(RobotConfig);
  SimRobot.UpdateConfig(RobotConfigNew);     // Here both the SimRobot.q and robot frames have already been updated.
  SimRobot.dq = RobotVelocity;
  // SimRobot.Update_J();
}

void RobotConfigToSimRobot(Robot & _SimRobot, const std::vector<double> &RobotConfig)
{
  std::vector<double >RobotVelocity(RobotConfig.size());
  RobotStateToSimRobot(_SimRobot, RobotConfig, RobotVelocity);
}

std::vector<LinkInfo> EndEffectorPND(const Robot & SimRobot, const vector<LinkInfo> & RobotLinkInfo, const SignedDistanceFieldInfo & SDFInfo)
{
  // This is a concise version of RobotEndEffectorInfo by only returning the positions and distances of end effector contacts.
  std::vector<LinkInfo> EndEffectorPNDInfo(RobotLinkInfo.size());
  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    int LinkiPNo = RobotLinkInfo[i].LocalContacts.size();
    EndEffectorPNDInfo[i].ContactPositions.reserve(LinkiPNo);
    EndEffectorPNDInfo[i].ContactDists.reserve(LinkiPNo);
    for (int j = 0; j < LinkiPNo; j++)
    {
      Vector3 LinkiPjPos;
      SimRobot.GetWorldPosition(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, LinkiPjPos);
      EndEffectorPNDInfo[i].ContactPositions.push_back(LinkiPjPos);
      EndEffectorPNDInfo[i].ContactDists.push_back(SDFInfo.SignedDistance(LinkiPjPos));
    }
  }
  return EndEffectorPNDInfo;
}

std::vector<LinkInfo> RobotEndEffectorInfo(const Robot &_SimRobot, const vector<LinkInfo>& _RobotLinkInfo, const SignedDistanceFieldInfo& _SDFInfo)
{
  // This function calculates the distance of the robot end effector to the environment expressed in Local Frame
  //  GetWorldPosition

  std::vector<LinkInfo> EndEffectorInfo(_RobotLinkInfo.size());

  LinkInfo RobotLinkInfo_i;
  int RobotLinkLocalPointNumber;
  Vector3 Link_i_Point_j_Position, Link_i_Point_j_Velocity, Link_i_Point_j_Dist;
  for (int i = 0; i < _RobotLinkInfo.size(); i++)
  {
    RobotLinkInfo_i = _RobotLinkInfo[i];
    RobotLinkLocalPointNumber = RobotLinkInfo_i.LocalContacts.size();

    EndEffectorInfo[i] = RobotLinkInfo_i;

    EndEffectorInfo[i].ContactPositions.reserve(RobotLinkLocalPointNumber);
    EndEffectorInfo[i].ContactVelocities.reserve(RobotLinkLocalPointNumber);
    EndEffectorInfo[i].ContactDists.reserve(RobotLinkLocalPointNumber);

    for (int j = 0; j < RobotLinkLocalPointNumber; j++)
    {
      _SimRobot.GetWorldPosition(RobotLinkInfo_i.LocalContacts[j], RobotLinkInfo_i.LinkIndex ,Link_i_Point_j_Position);
      EndEffectorInfo[i].ContactPositions.push_back(Link_i_Point_j_Position);
      _SimRobot.GetWorldVelocity(RobotLinkInfo_i.LocalContacts[j], RobotLinkInfo_i.LinkIndex ,_SimRobot.dq, Link_i_Point_j_Velocity);
      EndEffectorInfo[i].ContactVelocities.push_back(Link_i_Point_j_Velocity);
      EndEffectorInfo[i].ContactDists.push_back(_SDFInfo.SignedDistance(Link_i_Point_j_Position));
    }
  }
  return EndEffectorInfo;
}

void VectorPrintResult(const std::vector<double> & _vec)
{
  // This function is used to print the result of matrix
  for (int i = 0; i < _vec.size(); i++)
  {
      std::cout<<_vec[i]<<" ";
  }
  std::cout<<endl;
}

void MatrixPrintResult(const Matrix& _M)
{
  // This function is used to print the result of matrix
  for (int i = 0; i < _M.m; i++) {
    for (int j = 0; j < _M.n; j++) {
      std::cout<<_M(i,j)<<" ";
    }
    std::cout<<endl;
  }
  std::cout<<endl<<endl;
}

void MatrixPrintResult(const Matrix3& _M)
{
  // This function is used to print the result of matrix
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      std::cout<<_M(i,j)<<" ";
    }
    std::cout<<endl;
  }
  std::cout<<endl<<endl;
}


double RobotLinkLowerBound(Robot & SimRobot, const std::vector<double>& RobotState_ref, const SignedDistanceFieldInfo& _SDFInfo)
{
  // This function is used to calculate the lower bound of the robot link based on the current configuration

  std::vector<double> RobotConfig (RobotState_ref.begin(),  RobotState_ref.begin() + RobotState_ref.size()/2);
  Config RobotConfigNew(RobotConfig);
  SimRobot.UpdateConfig(RobotConfigNew);
  Vector3 Link_i_COM;
  std::vector<double> RobotLinkDist(RobotConfig.size()-6);
  for (int i = 0; i < RobotConfig.size()-6; i++)
  {
    SimRobot.links[i+6].GetWorldCOM(Link_i_COM);
    RobotLinkDist[i] = _SDFInfo.SignedDistance(Link_i_COM);
  }
  return *min_element(RobotLinkDist.begin(), RobotLinkDist.end());
}

std::vector<double> RandomVector(const int &length)
{
  // This function is used to shift the _Vector with a random coefficient multiplication

  std::uniform_real_distribution<double> unif(-1.0, 1.0);
  std::random_device rand_dev;          // Use random_device to get a random seed.
  std::mt19937 rand_engine(rand_dev()); // mt19937 is a good pseudo-random number generator.
  std::vector<double> _RandomVector(length);

  for (int i = 0; i < length; i++)
  {
    _RandomVector[i] = unif(rand_engine);

  }
  return _RandomVector;
}

void PIPPrint(const std::vector<PIPInfo>& PIPM)
{
  printf("Edges: %d ", PIPM.size());
  for (int i = 0; i < PIPM.size(); i++)
  {
    printf("PIP %d\n", i);
    printf("L: %f\n",         PIPM[i].L);
    printf("Ldot: %f\n",      PIPM[i].Ldot);
    printf("Theta: %f\n",     PIPM[i].theta);
    printf("Thetadot: %f\n",  PIPM[i].thetadot);
    printf("g: %f\n",         PIPM[i].g);
    printf("g angle: %f\n",   PIPM[i].g_angle);
  }
}

bool AreSame(const double& a, const double& b)
{
    return std::fabs(a - b) < std::numeric_limits<double>::epsilon();
}

std::vector<double> Opt_Seed_Zip(const int& VariableLength, const double &T_tot, const Eigen::MatrixXd& Q_Traj, const Eigen::MatrixXd& Qdot_Traj, const Eigen::MatrixXd& Qddot_Traj, const Eigen::MatrixXd& Lambda_Traj, const Eigen::MatrixXd& U_Traj)
{
  // This function is used to save
  std::vector<double> Opt_Seed;
  Opt_Seed.reserve(VariableLength);

  std::vector<double> Q_Traj_Vec(Q_Traj.data(), Q_Traj.data() + Q_Traj.rows() * Q_Traj.cols());
  std::vector<double> Qdot_Traj_Vec(Qdot_Traj.data(), Qdot_Traj.data() + Qdot_Traj.rows() * Qdot_Traj.cols());
  std::vector<double> Qddot_Traj_Vec(Qddot_Traj.data(), Qddot_Traj.data() + Qddot_Traj.rows() * Qddot_Traj.cols());
  std::vector<double> Lambda_Traj_Vec(Lambda_Traj.data(), Lambda_Traj.data() + Lambda_Traj.rows() * Lambda_Traj.cols());
  std::vector<double> U_Traj_Traj_Vec(U_Traj.data(), U_Traj.data() + U_Traj.rows() * U_Traj.cols());

  // Append Q_Traj_Vec
  Opt_Seed.insert(Opt_Seed.begin(), Q_Traj_Vec.begin(), Q_Traj_Vec.end());
  Opt_Seed.insert(Opt_Seed.begin() + Q_Traj_Vec.size(), Qdot_Traj_Vec.begin(), Qdot_Traj_Vec.end());
  Opt_Seed.insert(Opt_Seed.begin() + Q_Traj_Vec.size() + Qdot_Traj_Vec.size(), Qddot_Traj_Vec.begin(), Qddot_Traj_Vec.end());
  Opt_Seed.insert(Opt_Seed.begin() + Q_Traj_Vec.size() + Qdot_Traj_Vec.size() + Qddot_Traj_Vec.size(), Lambda_Traj_Vec.begin(), Lambda_Traj_Vec.end());
  Opt_Seed.insert(Opt_Seed.begin() + Q_Traj_Vec.size() + Qdot_Traj_Vec.size() + Qddot_Traj_Vec.size() + Lambda_Traj_Vec.size(), U_Traj_Traj_Vec.begin(), U_Traj_Traj_Vec.end());
  return Opt_Seed;
}

void Opt_Seed_Unzip(double &T_tot, std::vector<double>& Q_Traj_Vec, std::vector<double>& Qdot_Traj_Vec, std::vector<double>& Qddot_Traj_Vec, std::vector<double>& Lambda_Traj_Vec, std::vector<double>& U_Traj_Vec, const std::vector<double>& Opt_Seed, const int& DOF, const int& ContactPointNo, const int & GridsNo)
{
  const int Q_Traj_Length = DOF * GridsNo;
  const int Qdot_Traj_Length = Q_Traj_Length;
  const int Qddot_Traj_Length = Qdot_Traj_Length;
  const int Lambda_Traj_Length = 3 * ContactPointNo * GridsNo;        // Each force is a 3-d vector.
  const int U_Traj_Length = GridsNo * (DOF - 6);

  T_tot = Opt_Seed[0];

  int Opt_Seed_Begin_Index, Opt_Seed_End_Index;

  // Q_Traj
  Q_Traj_Vec.reserve(Q_Traj_Length);
  Opt_Seed_Begin_Index = 1;
  Opt_Seed_End_Index = Opt_Seed_Begin_Index + Q_Traj_Length;
  Q_Traj_Vec.insert(Q_Traj_Vec.begin(), Opt_Seed.begin() + Opt_Seed_Begin_Index, Opt_Seed.begin() + Opt_Seed_End_Index);

  // Qdot_Traj
  Qdot_Traj_Vec.reserve(Qdot_Traj_Length);
  Opt_Seed_Begin_Index = Opt_Seed_End_Index;
  Opt_Seed_End_Index = Opt_Seed_Begin_Index + Qdot_Traj_Length;
  Qdot_Traj_Vec.insert(Qdot_Traj_Vec.begin(), Opt_Seed.begin() + Opt_Seed_Begin_Index, Opt_Seed.begin() + Opt_Seed_End_Index);

  // Qddot_Traj
  Qddot_Traj_Vec.reserve(Qddot_Traj_Length);
  Opt_Seed_Begin_Index = Opt_Seed_End_Index;
  Opt_Seed_End_Index = Opt_Seed_Begin_Index + Qddot_Traj_Length;
  Qddot_Traj_Vec.insert(Qddot_Traj_Vec.begin(), Opt_Seed.begin() + Opt_Seed_Begin_Index, Opt_Seed.begin() + Opt_Seed_End_Index);

  // Lambda_Traj
  Lambda_Traj_Vec.reserve(Lambda_Traj_Length);
  Opt_Seed_Begin_Index = Opt_Seed_End_Index;
  Opt_Seed_End_Index = Opt_Seed_Begin_Index + Lambda_Traj_Length;
  Lambda_Traj_Vec.insert(Lambda_Traj_Vec.begin(), Opt_Seed.begin() + Opt_Seed_Begin_Index, Opt_Seed.begin() + Opt_Seed_End_Index);

  // U_Traj
  U_Traj_Vec.reserve(U_Traj_Length);
  Opt_Seed_Begin_Index = Opt_Seed_End_Index;
  Opt_Seed_End_Index = Opt_Seed_Begin_Index + U_Traj_Length;
  U_Traj_Vec.insert(U_Traj_Vec.begin(), Opt_Seed.begin() + Opt_Seed_Begin_Index, Opt_Seed.begin() + Opt_Seed_End_Index);
}

void RobotContactInfoUpdate(std::vector<ContactStatusInfo> & RobotContactInfo, const Robot & SimRobot, const vector<LinkInfo> & RobotLinkInfo, const SignedDistanceFieldInfo & SDFInfo)
{
  // This function is used to update robot's current contact information
  double epsTol = 0.02;     //2cm
  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    int LinkiPNo = RobotLinkInfo[i].LocalContacts.size();
    for (int j = 0; j < LinkiPNo; j++)
    {
      Vector3 LinkiPjPos;
      SimRobot.GetWorldPosition(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, LinkiPjPos);
      double LinkiPjDist = SDFInfo.SignedDistance(LinkiPjPos);
      if(LinkiPjDist<epsTol)
      {
        RobotContactInfo[i].LocalContactStatus[j] = 1;
      }
      else
      {
        RobotContactInfo[i].LocalContactStatus[j] = 0;
      }
    }
  }
}

std::vector<Vector3> ContactPositionFinder(const Robot& SimRobot, const std::vector<LinkInfo>& RobotLinkInfo, const std::vector<ContactStatusInfo>& RobotContactInfo)
{
  // This function finds robot's current active contact
  std::vector<Vector3> ActContacts;
  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    for (int j = 0; j < RobotLinkInfo[i].LocalContacts.size(); j++)
    {
      switch (RobotContactInfo[i].LocalContactStatus[j])
      {
        case 1:
        {
          // This means that current contact is active and we should keep its location and Jacobian.
          Vector3 LinkiPjPos;
          SimRobot.GetWorldPosition(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, LinkiPjPos);
          ActContacts.push_back(LinkiPjPos);
        }
        break;
        default:
        break;
      }
    }
  }
  return ActContacts;
}

std::vector<int> ActContactNJacobian(const Robot& SimRobot, const std::vector<LinkInfo>& RobotLinkInfo, const std::vector<ContactStatusInfo>& RobotContactInfo, std::vector<Vector3>& ActContacts, std::vector<Vector3>& ActVelocities, std::vector<Matrix> & ActJacobians, SignedDistanceFieldInfo & SDFInfo)
{
  // This function is used to get the robot's current active end effector position and Jacobian matrices.
  double DistTol = 0.025;
  std::vector<int> RealActiveIndices;
  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    for (int j = 0; j < RobotLinkInfo[i].LocalContacts.size(); j++)
    {
      switch (RobotContactInfo[i].LocalContactStatus[j])
      {
        case 1:
        {
          // This means that current contact is active and we should keep its location and Jacobian.
          Vector3 LinkiPjPos, LinkiPjVel;
          SimRobot.GetWorldPosition(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, LinkiPjPos);
          ActContacts.push_back(LinkiPjPos);

          SimRobot.GetWorldVelocity(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex ,SimRobot.dq, LinkiPjVel);
          ActVelocities.push_back(LinkiPjVel);

          double CurrentDist = SDFInfo.SignedDistance(LinkiPjPos);

          // if(CurrentDist>=DistTol)
          // {
          //   RealActiveIndices.push_back(0);
          // }
          // else
          // {
          //   RealActiveIndices.push_back(1);
          // }

          RealActiveIndices.push_back(1);
          Matrix ActJacobian;
          SimRobot.GetPositionJacobian(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, ActJacobian);
          ActJacobians.push_back(ActJacobian);
        }
        break;
        default:
        break;
      }
    }
  }
  return RealActiveIndices;
}

void ContactNumberFinder(const std::vector<ContactStatusInfo> & RobotContactInfo, int & InitContactNo, int & TotContactNo)
{
  InitContactNo = 0;
  TotContactNo = 0;
  for (int i = 0; i < RobotContactInfo.size(); i++)
  {
    int ContactStatus = *max_element(RobotContactInfo[i].LocalContactStatus.begin(), RobotContactInfo[i].LocalContactStatus.end());
    if(ContactStatus>0)
    {
      InitContactNo = InitContactNo + 1;
    }
    for (int j = 0; j < RobotContactInfo[i].LocalContactStatus.size(); j++)
    {
      TotContactNo = TotContactNo + 1;
    }
  }
}

int FileIndexFinder(bool UpdateFlag)
{
  // This function is used to read the current File Index to make sure that the file is ranged according to the number.
  string FileIndexName = "ConfigNo.txt";         // This file should be located in the "build" folder.
  ifstream FileIndexReader(FileIndexName);
  int FileIndex;
  string str_line;
  if (FileIndexReader.is_open())
  {
    while (getline (FileIndexReader,str_line) )
    {
      FileIndex = stoi(str_line);
    }
    FileIndexReader.close();
  }
  else cout << "Unable to open FileIndex file";

  // The second step is to update the next element value to be +1
  if(UpdateFlag)
  {
    const char *FileIndexWriter_Name = FileIndexName.c_str();
    std::ofstream FileIndexWriter;
    FileIndexWriter.open(FileIndexWriter_Name);
    FileIndexWriter<<std::to_string(FileIndex + 1)<<"\n";
    FileIndexWriter.close();
  }

  return FileIndex;
}

void CentroidalState(const Robot & SimRobot, Vector3 & COMPos, Vector3 & COMVel)
{
  // This function is used to get the centroidal position and velocity
  Vector3 COM = SimRobot.GetCOM();
  Matrix pCOMpq;
  SimRobot.GetCOMJacobian(pCOMpq);
  double COMVel_x =0.0, COMVel_y =0.0, COMVel_z =0.0;
  for (int i = 0; i < pCOMpq.n; i++)
  {
    COMVel_x = COMVel_x + pCOMpq(0,i) * SimRobot.dq[i];
    COMVel_y = COMVel_y + pCOMpq(1,i) * SimRobot.dq[i];
    COMVel_z = COMVel_z + pCOMpq(2,i) * SimRobot.dq[i];
  }
  Vector3 COMVel_i(COMVel_x, COMVel_y, COMVel_z);

  COMPos = COM;
  COMVel = COMVel_i;
}

double RandomValue(const double &bound)
{
  std::uniform_real_distribution<double> unif(-1.0 * bound, 1.0 * bound);
  std::random_device rand_dev;          // Use random_device to get a random seed.
  std::mt19937 rand_engine(rand_dev()); // mt19937 is a good pseudo-random number generator.
  double boundval = unif(rand_engine);
  return boundval;
}

int FallStatusFinder(const std::vector<double> & ObjTraj, const int & CutOffIndex)
{
  // This function is used to find whether there exist two less than 0 value in sequential order under CutOffIndex.
  std::vector<int> results;
  auto it = std::find_if(std::begin(ObjTraj), std::end(ObjTraj), [](double Obj_i){return Obj_i > 0.0;});
  while (it != std::end(ObjTraj))
  {
    results.emplace_back(std::distance(std::begin(ObjTraj), it));
    it = std::find_if(std::next(it), std::end(ObjTraj), [](double Obj_i){return Obj_i > 0.0;});
  }

  switch (results.size())
  {
    case 0:
    {
      return 0;
    }
    break;
    case 1:
    {
      return 0;
    }
    break;
    default:
    {
    }
    break;
  }

  // Now the job is to figure out the next two index
  for (int i = 0; i < results.size()-1; i++)
  {
    int FirstIndex = results[i];
    int SecondIndex = results[i+1];
    int Diff = SecondIndex - FirstIndex;
    switch (Diff)
    {
      case 1:
      {
        if(SecondIndex<=CutOffIndex)
        {
          return 1;
        }
      }
      break;
    }
  }
  return 0;
}

void ROCAppender(const double & TPR, const double & FPR, const int & CaseNumber, const int & CutOffIndex, const string FallDetector)
{
  // This function is used to generate the ROC curve
  string TPRTrajFile = FallDetector + "_" + std::to_string(CaseNumber) + "_" + std::to_string(CutOffIndex) + "_TPR.txt";
  const char *TPRTrajFile_Name = TPRTrajFile.c_str();

  std::ofstream TPRFile;
  TPRFile.open(TPRTrajFile_Name, std::ios_base::app);
  TPRFile<<std::to_string(TPR);
  TPRFile<<"\n";
  TPRFile.close();

  string FPRTrajFile = FallDetector + "_" + std::to_string(CaseNumber) + "_" + std::to_string(CutOffIndex) + "_FPR.txt";
  const char *FPRTrajFile_Name = FPRTrajFile.c_str();

  std::ofstream FPRFile;
  FPRFile.open(FPRTrajFile_Name, std::ios_base::app);
  FPRFile<<std::to_string(FPR);
  FPRFile<<"\n";
  FPRFile.close();
}

void Vector3Writer(const std::vector<Vector3> & ContactPoints, const std::string &ContactPointFileName)
{
  switch (ContactPoints.size())
  {
    case 0:
    {
      return;
    }
    break;
    default:
    break;
  }
  int NumberOfContactPoints = ContactPoints.size();
  std::vector<double> FlatContactPoints(3 * NumberOfContactPoints);
  int FlatContactPointIndex = 0;
  for (int i = 0; i < NumberOfContactPoints; i++)
  {
    FlatContactPoints[FlatContactPointIndex] = ContactPoints[i].x;
    FlatContactPointIndex++;
    FlatContactPoints[FlatContactPointIndex] = ContactPoints[i].y;
    FlatContactPointIndex++;
    FlatContactPoints[FlatContactPointIndex] = ContactPoints[i].z;
    FlatContactPointIndex++;
  }

  FILE * FlatContactPointsFile = NULL;
  string ContactPointFile = ContactPointFileName + ".bin";
  const char *ContactPointFile_Name = ContactPointFile.c_str();
  FlatContactPointsFile = fopen(ContactPointFile_Name, "wb");
  fwrite(&FlatContactPoints[0], sizeof(double), FlatContactPoints.size(), FlatContactPointsFile);
  fclose(FlatContactPointsFile);
  return;
}

std::vector<string>  EdgeFileNamesGene(const string & SpecificPath)
{
  std::vector<string> EdgeFileNames;
  string fEdgeAFile = SpecificPath + "EdgeATraj.txt";
  // const char *fEdgeAFile_Name = fEdgeAFile.c_str();
  string fEdgeBFile = SpecificPath + "EdgeBTraj.txt";
  // const char *fEdgeBFile_Name = fEdgeBFile.c_str();
  string fEdgeCOMFile = SpecificPath + "EdgeCOMTraj.txt";
  // const char *fEdgeCOMFile_Name = fEdgeCOMFile.c_str();
  string fEdgexTrajFile = SpecificPath + "EdgexTraj.txt";
  // const char *fEdgexTrajFile_Name = fEdgexTrajFile.c_str();
  string fEdgeyTrajFile = SpecificPath + "EdgeyTraj.txt";
  // const char *fEdgeyTrajFile_Name = fEdgeyTrajFile.c_str();
  string fEdgezTrajFile = SpecificPath + "EdgezTraj.txt";
  // const char *fEdgezTrajFile_Name = fEdgezTrajFile.c_str();

  EdgeFileNames.push_back(fEdgeAFile);
  EdgeFileNames.push_back(fEdgeBFile);
  EdgeFileNames.push_back(fEdgeCOMFile);
  EdgeFileNames.push_back(fEdgexTrajFile);
  EdgeFileNames.push_back(fEdgeyTrajFile);
  EdgeFileNames.push_back(fEdgezTrajFile);

  return EdgeFileNames;
}

std::vector<string>  EdgeFileNamesGene(const string & SpecificPath, const int & FileIndex)
{
  std::vector<string> EdgeFileNames;
  string fEdgeAFile = SpecificPath + std::to_string(FileIndex) + "/EdgeATraj.txt";
  // const char *fEdgeAFile_Name = fEdgeAFile.c_str();
  string fEdgeBFile = SpecificPath + std::to_string(FileIndex) + "/EdgeBTraj.txt";
  // const char *fEdgeBFile_Name = fEdgeBFile.c_str();
  string fEdgeCOMFile = SpecificPath + std::to_string(FileIndex) + "/EdgeCOMTraj.txt";
  // const char *fEdgeCOMFile_Name = fEdgeCOMFile.c_str();
  string fEdgexTrajFile = SpecificPath + std::to_string(FileIndex) + "/EdgexTraj.txt";
  // const char *fEdgexTrajFile_Name = fEdgexTrajFile.c_str();
  string fEdgeyTrajFile = SpecificPath + std::to_string(FileIndex) + "/EdgeyTraj.txt";
  // const char *fEdgeyTrajFile_Name = fEdgeyTrajFile.c_str();
  string fEdgezTrajFile = SpecificPath + std::to_string(FileIndex) + "/EdgezTraj.txt";
  // const char *fEdgezTrajFile_Name = fEdgezTrajFile.c_str();

  EdgeFileNames.push_back(fEdgeAFile);
  EdgeFileNames.push_back(fEdgeBFile);
  EdgeFileNames.push_back(fEdgeCOMFile);
  EdgeFileNames.push_back(fEdgexTrajFile);
  EdgeFileNames.push_back(fEdgeyTrajFile);
  EdgeFileNames.push_back(fEdgezTrajFile);

  return EdgeFileNames;
}

Vector3 ImpulForceGene(const double & ImpFx, const double & ImpFy, const double & ImpFz)
{
  // Default: ImpFx = 0.0, ImpFy = 10000.0, ImpFz = 0.0;
  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_real_distribution<> ImpXdis(0.75 * ImpFx, 1.25 * ImpFx);
  std::uniform_real_distribution<> ImpYdis(0.75 * ImpFy, 1.25 * ImpFy);
  std::uniform_real_distribution<> ImpZdis(0.5 * ImpFz, 1.0 * ImpFz);

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

  double Fx_t, Fy_t, Fz_t;

  Fx_t = Sign_x * ImpXdis(gen);
  Fy_t = Sign_y * ImpYdis(gen);
  Fz_t = Sign_z * ImpZdis(gen);

  return Vector3(Fx_t, Fy_t, Fz_t);
}

Vector3 ImpulForceMaxReader(const string & SpecificPath, const string & IFFileName)
{
  Vector3 IFMax(0.0, 0.0, 0.0);
  ifstream IFMaxFile (SpecificPath + IFFileName);
  std::vector<double> IFVec;
  if (IFMaxFile.is_open())
  {
    string str_line;
    while (getline (IFMaxFile, str_line) )
    {
      double ForceMag = 1.0 * stod(str_line);
      IFVec.push_back(ForceMag);
    }
    IFMaxFile.close();
  }
  else std::cerr << "\nUnable to open file " <<SpecificPath+IFFileName<<" does not exist!\n";

  if (IFVec.size() == 0)
  {
    std::cerr<<"\nImpulse Force Info failed to be loaded!"<<"\n";
  }
  IFMax.x = IFVec[0];
  IFMax.y = IFVec[1];
  IFMax.z = IFVec[2];

  return IFMax;
}

void PlanTimeRecorder(const double & PlanTimeVal, const string & SpecificPath)
{
  // This function is used to generate the ROC curve
  string PlanTimeFileStr = SpecificPath + "PlanTime.txt";
  const char *PlanTimeFile_Name = PlanTimeFileStr.c_str();

  std::ofstream PlanTimeFile;
  PlanTimeFile.open(PlanTimeFile_Name, std::ios_base::app);
  PlanTimeFile<<std::to_string(PlanTimeVal);
  PlanTimeFile<<"\n";
  PlanTimeFile.close();
}

bool FailureChecker(Robot & SimRobot, AnyCollisionGeometry3D & TerrColGeom, ReachabilityMap & RMObject, const double & DistTol)
{
  std::vector<double> LinkTerrDistVec;
  for (int i = 6; i < SimRobot.q.size(); i++)
  {
    if(!RMObject.EndEffectorIndices.count(i))
    {
      if(SimRobot.geometry[i]->Distance(TerrColGeom)<DistTol)
      {
        return true;
      }
    }
  }
  return false;
}

Vector3 ImpulseDirectionGene(Robot & SimRobotObj, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & RobotContactInfo)
{
  // This function is used to get the impulsive force direction such that the force can trigger the failure.
  std::vector<Vector3> SPVertices;
  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    int LinkiPNo = RobotLinkInfo[i].LocalContacts.size();
    for (int j = 0; j < LinkiPNo; j++)
    {
      switch (RobotContactInfo[i].LocalContactStatus[j])
      {
        case 1:
        {
          Vector3 LinkiPjPos;
          SimRobotObj.GetWorldPosition(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, LinkiPjPos);
          LinkiPjPos.z = 0.0;
          SPVertices.push_back(LinkiPjPos);
        }
        break;
        default:
        break;
      }
    }
  }

  Vector3 COM_Pos = SimRobotObj.GetCOM();
  int FacetFlag = 0;
  FacetInfo SPObj = FlatContactHullGeneration(SPVertices, FacetFlag);    // This is the support polygon
  COM_Pos.z = 0.0;
  std::vector<double> DistVec = SPObj.ProjPoint2EdgeDistVec(COM_Pos);
  std::vector<int> DistVecIndices(DistVec.size());
  for (int i = 0; i < DistVec.size(); i++)
  {
    DistVec[i] = DistVec[i] * DistVec[i];
    DistVecIndices[i] = i;
  }

  int MinIndex = std::distance(DistVec.begin(), std::min_element(DistVec.begin(), DistVec.end()));
  // It seems that a little randomness can increase complexity of the game!
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine e(seed);
  std::shuffle(std::begin(DistVecIndices), std::end(DistVecIndices), e);
  MinIndex = DistVecIndices[0];
  return -SPObj.EdgeNorms[MinIndex];
}

Vector3 FlatRandomDirection()
{
  double xDir = RandomValue(1.0);
  double yDir = RandomValue(1.0);
  Vector3 Dir(xDir, yDir, 0.0);
  Dir.setNormalized(Dir);
  return Dir;
}

void SDFWriter(const Meshing::VolumeGrid & SDFGrid, const string & Name)
{
  // This function is used to save the SDF into disk for debug purpose.
  // The estimated sizes of the environment
  double BB_x_min = SDFGrid.bb.bmin[0];
  double BB_x_max = SDFGrid.bb.bmax[0];

  double BB_y_min = SDFGrid.bb.bmin[1];
  double BB_y_max = SDFGrid.bb.bmax[1];

  double BB_z_min = SDFGrid.bb.bmin[2];
  double BB_z_max = SDFGrid.bb.bmax[2];

  double BB_x_length = BB_x_max - BB_x_min;
  double BB_y_length = BB_y_max - BB_y_min;
  double BB_z_length = BB_z_max - BB_z_min;

  double ExtCoeff = 0.125;

  // The estimated sizes of the environment
  double Envi_x_min = BB_x_min - ExtCoeff * BB_x_length;
  double Envi_x_max = BB_x_max + ExtCoeff * BB_x_length;

  double Envi_y_min = BB_y_min - ExtCoeff * BB_y_length;
  double Envi_y_max = BB_y_max + ExtCoeff * BB_y_length;

  double Envi_z_min = BB_z_min - ExtCoeff * BB_z_length;
  double Envi_z_max = BB_z_max + ExtCoeff * BB_z_length;

  double Envi_x_length = Envi_x_max - Envi_x_min;
  double Envi_y_length = Envi_y_max - Envi_y_min;
  double Envi_z_length = Envi_z_max - Envi_z_min;

  int xGrid = SDFGrid.value.m;
  int yGrid = SDFGrid.value.n;
  int zGrid = SDFGrid.value.p;

  double Envi_x_unit = Envi_x_length/(1.0* xGrid - 1.0);
  double Envi_y_unit = Envi_y_length/(1.0* yGrid - 1.0);
  double Envi_z_unit = Envi_z_length/(1.0* zGrid - 1.0);

  std::vector<double> Envi_x_coor(xGrid), Envi_y_coor(yGrid), Envi_z_coor(zGrid);

  // Here Envi_x/y/z_coor is the actual grids from the given environment
  for (int i = 0; i < xGrid; i++)
  {
    Envi_x_coor[i] = Envi_x_min + (1.0 * i) * Envi_x_unit;
  }
  for (int i = 0; i < yGrid; i++)
  {
    Envi_y_coor[i] = Envi_y_min + (1.0 * i) * Envi_y_unit;
  }
  for (int i = 0; i < zGrid; i++)
  {
    Envi_z_coor[i] = Envi_z_min + (1.0 * i) * Envi_z_unit;
  }
  // Generation of the SDFTensor structure
  Eigen::Tensor<double,3> SDFTensor(xGrid, yGrid, zGrid);
  SDFTensor.setZero();

  Vector3 GridPoint;
  double GridPointDist, GridPoint_x, GridPoint_y, GridPoint_z;
  std::vector<double> SDFVector;
  SDFVector.reserve(xGrid * yGrid * zGrid);
  for (int i = 0; i < xGrid; i++)
  {
    GridPoint_x = Envi_x_coor[i];
    for (int j = 0; j < yGrid; j++)
    {
      GridPoint_y = Envi_y_coor[j];
      for (int k = 0; k < zGrid; k++)
      {
        GridPoint_z = Envi_z_coor[k];
        GridPoint.set(GridPoint_x, GridPoint_y, GridPoint_z);
        GridPointDist = SDFGrid.TrilinearInterpolate(GridPoint);
        SDFTensor(i,j,k) = GridPointDist;
        SDFVector.push_back(GridPointDist);
      }
    }
  }

  std::vector<double> SDFSpecs;
  SDFSpecs.push_back(Envi_x_min);             SDFSpecs.push_back(Envi_x_max);
  SDFSpecs.push_back(Envi_y_min);             SDFSpecs.push_back(Envi_y_max);
  SDFSpecs.push_back(Envi_z_min);             SDFSpecs.push_back(Envi_z_max);

  SDFSpecs.push_back(Envi_x_unit);            SDFSpecs.push_back(Envi_y_unit);            SDFSpecs.push_back(Envi_z_unit);
  SDFSpecs.push_back(Envi_x_length);          SDFSpecs.push_back(Envi_y_length);          SDFSpecs.push_back(Envi_z_length);
  SDFSpecs.push_back(xGrid);
  SDFSpecs.push_back(yGrid);
  SDFSpecs.push_back(zGrid);

  // This function will write the computed SDFTensor and SDFSpecs into file
  FILE * SDFTensorFile = NULL;
  const string SDFDataPath ="./SDFTensor" + Name + ".bin";
  SDFTensorFile = fopen(SDFDataPath.c_str(), "wb");
  fwrite(&SDFVector[0], sizeof(double), SDFVector.size(), SDFTensorFile);
  fclose(SDFTensorFile);

  FILE * SDFSpecsFile = NULL;
  const string SDFSpecsPath ="./SDFSpecs" + Name + ".bin";
  SDFSpecsFile = fopen(SDFSpecsPath.c_str(), "wb");
  fwrite(&SDFSpecs[0], sizeof(double), SDFSpecs.size(), SDFSpecsFile);
  fclose(SDFSpecsFile);

  return;
}

int EndEffectorSelector(const std::vector<double> & ImpulseVec, const std::vector<double> & DistVec, const double & DisTol)
{
  std::vector<int> ValidEndEffector;
  std::vector<double> ValidImpulseEndEffector;
  for (int i = 0; i < DistVec.size(); i++)
  {
    if(DistVec[i]<DisTol)
    {
      ValidEndEffector.push_back(i);
      ValidImpulseEndEffector.push_back(ImpulseVec[i]);
    }
  }
  switch (ValidEndEffector.size())
  {
    case 0:
    {
      // Choose the one with the lowest signed distance to be modified.
      return std::distance(DistVec.begin(), std::min_element(DistVec.begin(), DistVec.end()));
    }
    break;
    default:
    {
      // Choose the one with the lowest impulse to be modified.
      int ValidEndEffectorIndex = std::distance(ValidImpulseEndEffector.begin(), std::min_element(ValidImpulseEndEffector.begin(), ValidImpulseEndEffector.end()));
      return ValidEndEffector[ValidEndEffectorIndex];
    }
    break;
  }
  return 0;
}

void PlanResWriter(const string & SpecificPath, const int & PushRecovFlag)
{
  const string PlanResStr = SpecificPath + "PlanRes.txt";
  const char *PlanResStr_Name = PlanResStr.c_str();
  std::ofstream PlanResWriter;
  PlanResWriter.open(PlanResStr_Name);
  PlanResWriter<<std::to_string(PushRecovFlag)<<"\n";
  PlanResWriter.close();

}
