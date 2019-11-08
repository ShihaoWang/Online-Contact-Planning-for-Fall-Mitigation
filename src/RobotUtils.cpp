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

          if(CurrentDist>=DistTol)
          {
            RealActiveIndices.push_back(0);
          }
          else
          {
            RealActiveIndices.push_back(1);
          }
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

int FileIndexFinder()
{
  // This function is used to read the current File Index to make sure that the file is ranged according to the number.
  string FileIndexName = "FileIndex.txt";         // This file should be located in the "build" folder.
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
  const char *FileIndexWriter_Name = FileIndexName.c_str();
  std::ofstream FileIndexWriter;
  FileIndexWriter.open(FileIndexWriter_Name);
  FileIndexWriter<<std::to_string(FileIndex + 1)<<"\n";
  FileIndexWriter.close();

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
