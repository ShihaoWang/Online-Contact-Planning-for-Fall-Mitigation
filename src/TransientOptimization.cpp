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

static double Tol = 1e-8;

struct TransientOpt: public NonlinearOptimizerInfo
{
  TransientOpt():NonlinearOptimizerInfo(){};

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
      std::vector<double> F_val = TransientOptNCons(*n, *neF, x_vec);
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
  static std::vector<double> TransientOptNCons(const int & nVar, const int & nObjNCons, const std::vector<double> & ActiveConfigOpt)
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
    SimRobotObj.GetWorldPosition(RobotLinkInfoObj[LinkInfoIndex].AvgLocalContact, RobotLinkInfoObj[LinkInfoIndex].LinkIndex, LinkiPjPos);
    double Avg_x_diff = LinkiPjPos.x - RefAvgPos.x;
    double Avg_y_diff = LinkiPjPos.y - RefAvgPos.y;
    double Avg_z_diff = LinkiPjPos.z - RefAvgPos.z;

    F[0] =  Avg_x_diff * Avg_x_diff + Avg_y_diff * Avg_y_diff + Avg_z_diff * Avg_z_diff;
    int ConstraintIndex = 1;
    for (int i = 0; i < RobotLinkInfoObj[LinkInfoIndex].LocalContacts.size(); i++)
    {
      SimRobotObj.GetWorldPosition(RobotLinkInfoObj[LinkInfoIndex].LocalContacts[i], RobotLinkInfoObj[LinkInfoIndex].LinkIndex, LinkiPjPos);
      F[ConstraintIndex] = SDFInfo.SignedDistance(LinkiPjPos);
      ConstraintIndex = ConstraintIndex + 1;
    }
    return F;
  }
};

int TransientOptFn(const Robot & SimRobot, const std::vector<double> & RefConfig, const int & _LinkInfoIndex, const Vector3 & RefPos, const std::vector<LinkInfo> & _RobotLinkInfo, ReachabilityMap & RMObject, std::vector<double> & RobotConfig, const int & Type)
{
  // This function is used to optimize robot's configuration such that a certain contact need to be made
  SimRobotObj = SimRobot;
  RefConfiguration = RefConfig;
  EndEffectorLink2Pivotal = RMObject.EndEffectorLink2Pivotal[_LinkInfoIndex];
  RefAvgPos = RefPos;
  RobotLinkInfoObj = _RobotLinkInfo;
  LinkInfoIndex = _LinkInfoIndex;

  TransientOpt TransientOptProblem;
  // Static Variable Substitution
  std::vector<int> ActiveLinkChain = RMObject.EndEffectorLink2Pivotal[_LinkInfoIndex];
  std::vector<double> ActiveJoint(ActiveLinkChain.size());
  int n = ActiveLinkChain.size();

  // Cost function on the norm difference between the reference avg position and the modified contact position.
  int neF = 1;
  neF = neF + RobotLinkInfoObj[LinkInfoIndex].LocalContacts.size();
  TransientOptProblem.InnerVariableInitialize(n, neF);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> xlow_vec(n), xupp_vec(n);

  for (int i = 0; i < n; i++)
  {
    // Configuration
    xlow_vec[i] = SimRobot.qMin(ActiveLinkChain[i]);
    xupp_vec[i] = SimRobot.qMax(ActiveLinkChain[i]);
    ActiveJoint[i] = RefConfig[ActiveLinkChain[i]];
  }
  TransientOptProblem.VariableBoundsUpdate(xlow_vec, xupp_vec);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> Flow_vec(neF), Fupp_vec(neF);
  Flow_vec[0] = 0;
  Fupp_vec[0] = 1e20;           // The default idea is that the objective should always be nonnegative
  int ConstraintIndex = 1;
  switch (Type)
  {
    case 1:
    {
      // This means that the current contact has to be active.
      for (int i = 0; i < RobotLinkInfoObj[LinkInfoIndex].LocalContacts.size(); i++)
      {
        Flow_vec[ConstraintIndex] = 0;
        Fupp_vec[ConstraintIndex] = 0;
        ConstraintIndex = ConstraintIndex + 1;
      }
    }
    break;
    default:
    {
      // This means that the current contact has to have positive margin.
      for (int i = 0; i < RobotLinkInfoObj[LinkInfoIndex].LocalContacts.size(); i++)
      {
        Flow_vec[ConstraintIndex] = 0;
        Fupp_vec[ConstraintIndex] = 1e10;
        ConstraintIndex = ConstraintIndex + 1;
      }
    }
    break;
  }

  TransientOptProblem.ConstraintBoundsUpdate(Flow_vec, Fupp_vec);

  /*
    Initialize the seed guess
  */
  TransientOptProblem.SeedGuessUpdate(ActiveJoint);

  /*
    Given a name of this problem for the output
  */
  TransientOptProblem.ProblemNameUpdate("TransientOptProblem", 0);

  // Here we would like allow much more time to be spent on IK
  TransientOptProblem.NonlinearProb.setIntParameter("Iterations limit", 100);
  TransientOptProblem.NonlinearProb.setIntParameter("Major iterations limit", 15);
  TransientOptProblem.NonlinearProb.setIntParameter("Major print level", 0);
  TransientOptProblem.NonlinearProb.setIntParameter("Minor print level", 0);
  /*
    ProblemOptions seting
  */
  // Solve with Finite-Difference
  TransientOptProblem.ProblemOptionsUpdate(0, 3);
  TransientOptProblem.Solve(ActiveJoint);

  for (int i = 0; i < EndEffectorLink2Pivotal.size(); i++)
  {
    RefConfiguration[EndEffectorLink2Pivotal[i]] = ActiveJoint[i];
  }
  RobotConfig = RefConfiguration;
  Config ConfigOptNew(RefConfiguration);
  SimRobotObj.UpdateConfig(ConfigOptNew);
  std::vector<double> Fcon(neF-1);
  ConstraintIndex = 0;
  double DisCoeff = 0.0;
  switch (Type)
  {
    case 1:
    {
      DisCoeff = 1.0;
    }
    break;
    default:
    {

    }
    break;
  }
  for (int i = 0; i < RobotLinkInfoObj[LinkInfoIndex].LocalContacts.size(); i++)
  {
    Vector3 LinkiPjPos;
    SimRobotObj.GetWorldPosition(RobotLinkInfoObj[LinkInfoIndex].LocalContacts[i], RobotLinkInfoObj[LinkInfoIndex].LinkIndex, LinkiPjPos);
    double SDF_i = NonlinearOptimizerInfo::SDFInfo.SignedDistance(LinkiPjPos);
    Fcon[ConstraintIndex] = DisCoeff * SDF_i * SDF_i;
    ConstraintIndex = ConstraintIndex + 1;
  }

  double Fconval = *std::min_element(Fcon.begin(),Fcon.end());
  int OptRes = -1;
  if(Fconval<Tol)
  {
    OptRes = 1;
  }
  return OptRes;
}
