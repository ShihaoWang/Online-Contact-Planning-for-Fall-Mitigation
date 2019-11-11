#ifndef COMMON_HEADER_H
#define COMMON_HEADER_H
#include <iostream>
#include <fstream>
#include <limits.h>
#include <string>
#include "RobotInfo.h"
/* 0. Robot Info Initiaization */
std::vector<LinkInfo> ContactInfoLoader(const string & ContactLinkFile, int & ContactPointNo);
std::vector<ContactStatusInfo> ContactStatusInfoLoader(const string & ContactStatusFile);

/* 1. Signed Distance Field */
SignedDistanceFieldInfo SignedDistanceFieldGene(const RobotWorld& WorldObj, const int& GridsNo);
SignedDistanceFieldInfo SignedDistanceFieldLoader(const int GridsNo);

/* 2. Robot State File Operations */
void RobotConfigLoader(Robot &SimRobot, const string &user_path, const string &file_name);
void RobotStateLoader(Robot &SimRobot, const string &user_path, const string &config_file_name, const string &velo_file_name);
void RobotConfigWriter(const std::vector<double> & Config, const string &user_path, const string &config_file_name);
void RobotStateWriter(const std::vector<double> &Config, const std::vector<double> &Velocity, const string &user_path, const string &config_file_name, const string &velo_file_name);
void ConvexEdgesWriter(const std::vector<FacetInfo>& FacetInfoObj, const string &user_path, const string &edge_file_name);
void PIPsWriter(const std::vector<PIPInfo>& PIPInfoTotal, const string &user_path, const string &edge_file_name);
void VectorWriter(const std::vector<double> & Cost_Vec, const string &user_path, const string &config_file_name);
void TrajAppender(const char * qTrajFile_Name, const Config & Traj_i, const int & DOF);
void SpecsWriter(const Robot & SimRobot, const double & t_final, const double & t_last, const Vector3 & F_t, const int & InitContactNo, const int & FileIndex);
void CentroidalFailureMetricWriter(const Vector3 & COM, const Vector3 & COMVel, const double & KE, const std::vector<double> FailureMetricVec, const std::vector<const char*> & CentroidalFileNames, const std::vector<const char*> & FailureMetricNames);
void COMDesWriter(const int & FileIndex, const Vector3 & COMPosdes);
void IntersectionsWriter(const std::vector<Vector3> & Intersections, const string &user_path, const string &inters_file_name);

/* 3. Robot Initial State Optimization */
bool InitialStateOptFn(Robot& _SimRobotObj, const std::vector<LinkInfo> & _RobotLinkInfo, const std::vector<ContactStatusInfo> &  _RobotContactInfo, const SignedDistanceFieldInfo& _SDFInfo, const std::vector<double>& _RobotConfigRef, const double & _KEInit, const Vector3& _CentDirection, std::vector<double> & RobotConfig, std::vector<double> & RobotVelocity, const bool & ConfigFlag, const bool & VelocityFlag);

/* 4. Robot Utility Functions */
void SimRobotToRobotState(const Robot &_SimRobot, std::vector<double>& _Config, std::vector<double>& _Velocity);
std::vector<double> SimRobotToRobotState(const Robot &_SimRobot);
std::vector<LinkInfo> EndEffectorPND(const Robot &SimRobot, const vector<LinkInfo>& RobotLinkInfo, const SignedDistanceFieldInfo& SDFInfo);
std::vector<LinkInfo> RobotEndEffectorInfo(const Robot &_SimRobot, const vector<LinkInfo>& _RobotLinkInfo, const SignedDistanceFieldInfo& _SDFInfo);
void MatrixPrintResult(const Matrix& _M);
void VectorPrintResult(const std::vector<double> & _vec);
double RobotLinkLowerBound(Robot & SimRobot, const std::vector<double>& RobotState_ref, const SignedDistanceFieldInfo& _SDFInfo);
void RobotStateToSimRobot(Robot & SimRobot, const std::vector<double> & RobotState);
void RobotStateToSimRobot(Robot & SimRobot, const std::vector<double> &RobotConfig, const std::vector<double> & RobotVelocity);
void RobotConfigToSimRobot(Robot & _SimRobot, const std::vector<double> &RobotConfig);
std::vector<double> RandomVector(const int &length);
void PIPPrint(const std::vector<PIPInfo>& PIPM);
std::vector<double> Opt_Seed_Zip(const int& VariableLength, const double &T_tot, const Eigen::MatrixXd& Q_Traj, const Eigen::MatrixXd& Qdot_Traj, const Eigen::MatrixXd& Qddot_Traj, const Eigen::MatrixXd& Lambda_Traj, const Eigen::MatrixXd& U_Traj);
void Opt_Seed_Unzip(double &T_tot, std::vector<double>& Q_Traj_Vec, std::vector<double>& Qdot_Traj_Vec, std::vector<double>& Qddot_Traj_Vec, std::vector<double>& Lambda_Traj_Vec, std::vector<double>& U_Traj_Vec, const std::vector<double>& Opt_Seed, const int& DOF, const int& ContactPointNo, const int & GridsNo);
std::vector<int> ActContactNJacobian(const Robot& SimRobot, const std::vector<LinkInfo>& RobotLinkInfo, const std::vector<ContactStatusInfo>& RobotContactInfo, std::vector<Vector3>& ActContacts, std::vector<Vector3>& ActVelocities, std::vector<Matrix> & ActJacobians, SignedDistanceFieldInfo & SDFInfo);
double RandomValue(const double &bound);
void ContactNumberFinder(const std::vector<ContactStatusInfo> & RobotContactInfo, int & InitContactNo, int & TotContactNo);
int FileIndexFinder();
void CentroidalState(const Robot & SimRobot, Vector3 & COMPos, Vector3 & COMVel);
void RobotContactInfoUpdate(std::vector<ContactStatusInfo> & RobotContactInfo, const Robot & SimRobot, const vector<LinkInfo> & RobotLinkInfo, const SignedDistanceFieldInfo & SDFInfo);
int FallStatusFinder(const std::vector<double> & ObjTraj, const int & CutOffIndex);
void ROCAppender(const double & TPR, const double & FPR, const int & CaseNumber, const int & CutOffIndex, const string FallDetector);


/* 5. Contact Polyhedron functions */
FacetInfo FlatContactHullGeneration(const std::vector<Vector3> & _CPVertices, int& FacetFlag);
std::vector<Vector3> ContactPolyhedronVertices(const Robot & SimRobot,const std::vector<LinkInfo> &RobotLinkInfo, const SignedDistanceFieldInfo& SDFInfo);
std::vector<FacetInfo> ContactHullGeneration(const std::vector<Vector3>& _CPVertices, std::vector<Vector3> & CPVertex, std::vector<Vector3> & CPEdgeA, std::vector<Vector3> & CPEdgeB);
int CollinearTest(const std::vector<Vector3> & _CPVertices);
void ConeUnitGenerator(const std::vector<Vector3> & ActContacts, SignedDistanceFieldInfo& SDFInfo, std::vector<Vector3> & ConeAllUnit, std::vector<Vector3> & ConeUnits, const int & edge_no, const double & mu);
std::vector<PIPInfo> PIPGenerator(const std::vector<Vector3> & CPVertices, const Vector3 & COMPos, const Vector3 & COMVel);
double RBGenerator(const std::vector<PIPInfo> & PIPTotal, int & PIPIndex);
double CapturePointGenerator(const std::vector<PIPInfo> & PIPTotal, int & PIPIndex);
double ZMPGeneratorAnalysis(const std::vector<PIPInfo> & PIPTotal, const Vector3 & COMPos, const Vector3 & COMAcc, const double & Margin);
std::vector<Vector3> FullPIPInterCal(const std::vector<FacetInfo> & FacetInfoObj, const Vector3 & COM);
void ContactPolytopeWriter(const std::vector<PIPInfo> & PIPTotal, const std::vector<const char*> & EdgeFileNames);

/* 8. Simulation Test */
void SimulationTest(WorldSimulation & Sim, std::vector<LinkInfo> & RobotLinkInfo, std::vector<ContactStatusInfo> & RobotContactInfo, SignedDistanceFieldInfo & SDFInfo, SimGUIBackend & Backend, const std::vector<Vector3> & ContactPositionRef, const double & dt, const int & FileIndex);

/* 9. Stabilizing Controller */
std::vector<double> StabilizingControllerContact(const Robot& SimRobot, const std::vector<Matrix> & _ActJacobians, const std::vector<Vector3>& _ConeAllUnits, const int & _EdgeNo, const int& _DOF, const double& dt, std::vector<Config>& qTrajDes, std::vector<Config> & qdotTrajDes, std::vector<Config> & qTrajAct, std::vector<Config> & qdotTrajAct, std::vector<LinkInfo> & _RobotLinkInfo, std::vector<ContactStatusInfo> & _RobotContactInfo, const std::vector<Vector3> & _ContactPositionsRef, std::vector<Vector3> & _ContactPositions, std::vector<Vector3> & _ContactVelocities, const int & _ContactPointNo, const int & StepIndex);

void RobotStateLoader(const string &user_path, const string &config_file_name, const string &velo_file_name, std::vector<double> & RobotConfig, std::vector<double> & RobotVelocity);

#endif
