#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include <omp.h>
#include "NonlinearOptimizerInfo.h"

static Robot SimRobotObj;
static std::vector<double> RefConfiguration;
static std::vector<int> EndEffectorLink2Pivotal;
static Vector3 RefAvgPos;
static std::vector<LinkInfo> RobotLinkInfoObj;
static int LinkInfoIndex;

struct ContactFeasibleOpt: public NonlinearOptimizerInfo
{
  ContactFeasibleOpt():NonlinearOptimizerInfo(){};

  // This struct inherits the NonlinearOptimizerInfo struct and we just need to defined the Constraint function
  static void ObjNConstraint(int    *Status, int *n,    double x[],
    int    *needF,  int *neF,  double F[],
    int    *needG,  int *neG,  double G[],
    char      *cu,  int *lencu,
    int    iu[],    int *leniu,
    double ru[],    int *lenru)
    {
      std::vector<double> x_vec(*n);
      for (int i = 0; i < *n; i++)
      {
        x_vec[i] = x[i];
      }
      std::vector<double> F_val = ContactFeasibleOptNCons(*n, *neF, x_vec);
      for (int i = 0; i < *neF; i++)
      {
        F[i] = F_val[i];
      }
    }
  void Solve(std::vector<double> &RobotConfig)
  {
    int StartType = 0;
    NonlinearProb.solve(StartType, neF, n, ObjAdd, ObjRow, ObjNConstraint,
      xlow, xupp, Flow, Fupp,
      x, xstate, xmul, F, Fstate, Fmul,
      nS, nInf, sumInf);
      for (int i = 0; i < n; i++)
      {
        RobotConfig[i] = x[i];
      }
      delete []x;      delete []xlow;   delete []xupp;
      delete []xmul;   delete []xstate;

      delete []F;      delete []Flow;   delete []Fupp;
      delete []Fmul;   delete []Fstate;
  }
  static std::vector<double> ContactFeasibleOptNCons(const int & nVar, const int & nObjNCons, const std::vector<double> & ActiveConfigOpt)
  {
    // This funciton provides the constraint for the configuration variable
    std::vector<double> F(nObjNCons);
    for (int i = 0; i < EndEffectorLink2Pivotal.size(); i++)
    {
      RefConfiguration[EndEffectorLink2Pivotal[i]] = ActiveConfigOpt[i];
    }
    Config ConfigOptNew(RefConfiguration);
    SimRobotObj.UpdateConfig(ConfigOptNew);

    double ConfigVia = 0.0;

    Vector3 LinkiPjPos;
    SimRobotObj.GetWorldPosition(RobotLinkInfoObj[LinkInfoIndex].AvgLocalContact, LinkInfoIndex, LinkiPjPos);
    double Avg_x_diff = LinkiPjPos.x - RefAvgPos.x;
    double Avg_y_diff = LinkiPjPos.y - RefAvgPos.y;
    double Avg_z_diff = LinkiPjPos.z - RefAvgPos.z;

    F[0] =  Avg_x_diff * Avg_x_diff + Avg_y_diff * Avg_y_diff + Avg_z_diff * Avg_z_diff;
    int ConstraintIndex = 1;
    for (int i = 0; i < RobotLinkInfoObj[LinkInfoIndex].LocalContacts.size(); i++)
    {
      SimRobotObj.GetWorldPosition(RobotLinkInfoObj[LinkInfoIndex].LocalContacts[i], LinkInfoIndex, LinkiPjPos);
      F[ConstraintIndex] = SDFInfo.SignedDistance(LinkiPjPos);
      ConstraintIndex = ConstraintIndex + 1;
    }
    return F;
  }
};

int ContactFeasibleOptFn(const Robot& SimRobot, const int & _LinkInfoIndex, const Vector3 & RefPos, const std::vector<LinkInfo> & _RobotLinkInfo, ReachabilityMap & RMObject, std::vector<double> &RobotConfig)
{
  // This function is used to optimize robot's configuration such that a certain contact need to be made
  SimRobotObj = SimRobot;
  RefConfiguration = SimRobot.q;
  EndEffectorLink2Pivotal = RMObject.EndEffectorLink2Pivotal[LinkInfoIndex];
  RefAvgPos = RefPos;
  RobotLinkInfoObj = _RobotLinkInfo;
  LinkInfoIndex = _LinkInfoIndex;

  ContactFeasibleOpt ContactFeasibleOptProblem;
  // Static Variable Substitution
  std::vector<int> ActiveLinkChain = RMObject.EndEffectorLink2Pivotal[LinkInfoIndex];
  std::vector<double> ActiveJoint(ActiveLinkChain.size());
  int n = ActiveLinkChain.size();
  // Cost function on the norm difference between the reference avg position and the modified contact position.
  int neF = 1;
  neF = neF + RobotLinkInfoObj[LinkInfoIndex].LocalContacts.size();
  ContactFeasibleOptProblem.InnerVariableInitialize(n, neF);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> xlow_vec(n), xupp_vec(n);

  for (int i = 0; i < n; i++)
  {
    // Configuration
    xlow_vec[i] = SimRobot.qMin(ActiveLinkChain[i]);
    xupp_vec[i] = SimRobot.qMax(ActiveLinkChain[i]);
    ActiveJoint[i] = SimRobot.q[ActiveLinkChain[i]];
  }
  ContactFeasibleOptProblem.VariableBoundsUpdate(xlow_vec, xupp_vec);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> Flow_vec(neF), Fupp_vec(neF);
  Flow_vec[0] = 0;
  Fupp_vec[0] = 1e20;           // The default idea is that the objective should always be nonnegative
  int ConstraintIndex = 1;
  for (int i = 0; i < RobotLinkInfoObj[LinkInfoIndex].LocalContacts.size(); i++)
  {
    Flow_vec[ConstraintIndex] = 0;
    Fupp_vec[ConstraintIndex] = 0;
    ConstraintIndex = ConstraintIndex + 1;
  }
  ContactFeasibleOptProblem.ConstraintBoundsUpdate(Flow_vec, Fupp_vec);

  /*
    Initialize the seed guess
  */
  ContactFeasibleOptProblem.SeedGuessUpdate(ActiveJoint);

  /*
    Given a name of this problem for the output
  */
  ContactFeasibleOptProblem.ProblemNameUpdate("ContactFeasibleOptProblem", 0);

  // Here we would like allow much more time to be spent on IK
  ContactFeasibleOptProblem.NonlinearProb.setIntParameter("Iterations limit", 100);
  ContactFeasibleOptProblem.NonlinearProb.setIntParameter("Major iterations limit", 25);
  ContactFeasibleOptProblem.NonlinearProb.setIntParameter("Major print level", 0);
  ContactFeasibleOptProblem.NonlinearProb.setIntParameter("Minor print level", 0);
  /*
    ProblemOptions seting
  */
  // Solve with Finite-Difference
  ContactFeasibleOptProblem.ProblemOptionsUpdate(0, 3);
  ContactFeasibleOptProblem.Solve(ActiveJoint);
  return 0;
}

static void StepIntegrator(double & Theta, double & Thetadot, Vector3 & COMPos, Vector3 & COMVel, const PIPInfo & PIPObj, double & dt)
{
  // This function is used to integrate robot's PIP dynamics.
  double L = PIPObj.L;
  double g = PIPObj.g;
  // Integration with the assumption that robot's acceleration remains to be constant during dt.
  double Thetaddot = g/L * sin(Theta);
  Theta = Theta + Thetadot * dt + 0.5 * Thetaddot * dt * dt;
  Thetadot = Thetadot + Thetaddot * dt;
  COMPos = L * cos(Theta) * PIPObj.y_prime_unit - L * sin(Theta) * PIPObj.z_prime_unit + PIPObj.Intersection;
  Vector3 COMVelDirMag = -cross(PIPObj.x_prime_unit, COMPos - PIPObj.Intersection);
  Vector3 COMVelDir;
  COMVelDirMag.getNormalized(COMVelDir);
  COMVel = L * Thetadot * COMVelDir;
}

// This file saves functions needed for contact planning.
double CollisionTimeEstimator(const Vector3 & EdgeA, const Vector3 & EdgeB, const Vector3 & COMPos, const Vector3 & COMVel, SignedDistanceFieldInfo & SDFInfo, std::vector<Vector3> & COMPosTraj, std::vector<Vector3> & COMVelTraj, int & CollisionIndex)
{
  // This function is used to estimate the collision time for the robot.
  PIPInfo PIPObj = PIPGeneratorInner(EdgeA, EdgeB, COMPos, COMVel);
  // Then the robot is assumed to be rotating along PIP motion.
  // Whose equation of motion should be thetaddot = g*/L * sin(theta)

  // According to the observation, the robot's at most takes 2s to fall to the ground.
  double MaximumTime = 2.0;
  double CollisionTime = 0.0;
  double dt = 0.05;      // Each time step takes 0.025s.
  double Theta = PIPObj.theta;
  double Thetadot = PIPObj.thetadot;
  int MaximumDataPoint = floor(MaximumTime/dt);
  CollisionIndex = 0;
  // std::vector<double> ThetaTraj, ThetadotTraj;
  COMPosTraj.reserve(MaximumDataPoint);
  COMVelTraj.reserve(MaximumDataPoint);
  while (CollisionTime<MaximumTime)
  {
    Vector3 COMPosNew, COMVelNew;
    StepIntegrator(Theta, Thetadot, COMPosNew, COMVelNew, PIPObj, dt);
    // ThetaTraj.push_back(Theta);
    // ThetadotTraj.push_back(Thetadot);
    COMPosTraj.push_back(COMPosNew);
    COMVelTraj.push_back(COMVelNew);
    CollisionTime+=dt;
    // Computed COM position
    double CurrentDist = SDFInfo.SignedDistance(COMPosNew);
    if(CurrentDist<0)
    {
      break;
    }
    CollisionIndex++;

  }
  // No matter what has happended, this while loop has to be terminated.
  return CollisionTime;
}

double ContactModiPreEstimation(Robot & SimRobot, const PIPInfo & PIPObj, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & RobotContactInfo, SignedDistanceFieldInfo & SDFInfo, int & FixerInfoIndex, std::vector<Vector3> & COMPosTraj, std::vector<Vector3> & COMVelTraj)
{
  Vector3 EdgeA = PIPObj.EdgeA;
  Vector3 EdgeB = PIPObj.EdgeB;

  int EdgeAEndInfoIndex = Edge2EndEffector(SimRobot, EdgeA, RobotLinkInfo);
  int EdgeBEndInfoIndex = Edge2EndEffector(SimRobot, EdgeB, RobotLinkInfo);

  // Here the high level contact planning leads to two consequences: Contact Modification and Contact Addition

  // 1. Contact Modification Analysis
  FixerInfoIndex = -1;
  Vector3 RotEdgeOri, RotEdgeGoal;
  if(EdgeAEndInfoIndex == EdgeBEndInfoIndex)
  {
    // This means that EdgeA and EdgeB are on the same end effector so the other three end effectors can be used for contact planning.
    FixerInfoIndex = EdgeAEndInfoIndex;
    RotEdgeOri = EdgeA;
    RotEdgeGoal = EdgeB;
  }
  else
  {
    // This indicates the current end is owned by two end effectors so we should be more careful about the selection of which end effector to be modified.
    // Basically the main idea is to alter the link where the rest of the link has a lower failure metric.
    Vector3 RotKeepBOri, RotKeepBGoal;
    double KeepBFixedCost = FailureMetricwithContactChange(SimRobot, EdgeAEndInfoIndex, RobotLinkInfo, RobotContactInfo, RotKeepBOri, RotKeepBGoal);
    Vector3 RotKeepAOri, RotKeepAGoal;
    double KeepAFixedCost = FailureMetricwithContactChange(SimRobot, EdgeBEndInfoIndex, RobotLinkInfo, RobotContactInfo, RotKeepAOri, RotKeepAGoal);

    if(KeepBFixedCost>KeepAFixedCost)
    {
      // Here A should be kept fixed.
      FixerInfoIndex = EdgeAEndInfoIndex;
      RotEdgeOri = RotKeepAOri;
      RotEdgeGoal = RotKeepAGoal;
    }
    else
    {
      // Here B should be kept fixed.
      FixerInfoIndex = EdgeBEndInfoIndex;
      RotEdgeOri = RotKeepBOri;
      RotEdgeGoal = RotKeepBGoal;
    }
  }
  /* Robot's COMPos and COMVel */
  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(SimRobot, COMPos, COMVel);
  std::vector<Vector3> PredCOMPosTraj, PredCOMVelTraj;
  int CollisionIndex;
  double CollisionTime = CollisionTimeEstimator(RotEdgeOri, RotEdgeGoal, COMPos, COMVel, SDFInfo, PredCOMPosTraj, PredCOMVelTraj, CollisionIndex);

  COMPosTraj.reserve(CollisionIndex);
  COMVelTraj.reserve(CollisionIndex);
  for (int i = 0; i < CollisionIndex; i++)
  {
    COMPosTraj.push_back(PredCOMPosTraj[i]);
    COMVelTraj.push_back(PredCOMVelTraj[i]);
  }
  return CollisionTime;
}

double ContactAddPreEstimation(Robot & SimRobot, const PIPInfo & PIPObj, SignedDistanceFieldInfo & SDFInfo, std::vector<Vector3> & COMPosTraj, std::vector<Vector3> & COMVelTraj)
{
  Vector3 EdgeA = PIPObj.EdgeA;
  Vector3 EdgeB = PIPObj.EdgeB;

  /* Robot's COMPos and COMVel */
  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  CentroidalState(SimRobot, COMPos, COMVel);
  std::vector<Vector3> PredCOMPosTraj, PredCOMVelTraj;
  int CollisionIndex;
  double CollisionTime = CollisionTimeEstimator(EdgeA, EdgeB, COMPos, COMVel, SDFInfo, PredCOMPosTraj, PredCOMVelTraj, CollisionIndex);
  COMPosTraj.reserve(CollisionIndex);
  COMVelTraj.reserve(CollisionIndex);
  for (int i = 0; i < CollisionIndex; i++)
  {
    COMPosTraj.push_back(PredCOMPosTraj[i]);
    COMVelTraj.push_back(PredCOMVelTraj[i]);
  }
  return CollisionTime;
}
