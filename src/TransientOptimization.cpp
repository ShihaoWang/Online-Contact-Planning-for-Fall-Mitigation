#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include <omp.h>
#include "NonlinearOptimizerInfo.h"

static Robot SimRobotObj;
static int SwingLimbIndex;
static std::vector<int> SwingLimbChain;
static Vector3 PosGoal;
static std::vector<double> RefConfig;
static double PosGoalDist;

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
    for (int i = 0; i < SwingLimbChain.size(); i++)
    {
      RefConfig[SwingLimbChain[i]] = ActiveConfigOpt[i];
    }
    SimRobotObj.UpdateConfig(Config(RefConfig));

    double ConfigVia = 0.0;

    Vector3 LinkiPjPos;
    SimRobotObj.GetWorldPosition(NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].AvgLocalContact, NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LinkIndex, LinkiPjPos);
    double Avg_x_diff = LinkiPjPos.x - PosGoal.x;
    double Avg_y_diff = LinkiPjPos.y - PosGoal.y;
    double Avg_z_diff = LinkiPjPos.z - PosGoal.z;

    F[0] =  Avg_x_diff * Avg_x_diff + Avg_y_diff * Avg_y_diff + Avg_z_diff * Avg_z_diff;
    int ConstraintIndex = 1;
    for (int i = 0; i < NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LocalContacts.size(); i++)
    {
      SimRobotObj.GetWorldPosition(NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LocalContacts[i], NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LinkIndex, LinkiPjPos);
      F[ConstraintIndex] = SDFInfo.SignedDistance(LinkiPjPos) - PosGoalDist;
      ConstraintIndex = ConstraintIndex + 1;
    }
    return F;
  }
};

std::vector<double> TransientOptFn(const Robot & SimRobot, const int & _SwingLimbIndex, const Vector3 & _PosGoal, ReachabilityMap & RMObject, bool & OptFlag)
{
  // This function is used to optimize robot's configuration such that a certain contact can be reached for that end effector.
  SimRobotObj = SimRobot;
  SwingLimbIndex = _SwingLimbIndex;
  SwingLimbChain = RMObject.EndEffectorLink2Pivotal[_SwingLimbIndex];
  PosGoal = _PosGoal;
  PosGoalDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(PosGoal);
  PosGoalDist = max(0.0, PosGoalDist);
  RefConfig = SimRobot.q;
  OptFlag = true;

  TransientOpt TransientOptProblem;

  // Static Variable Substitution
  std::vector<double> InitSwingLimbChain(SwingLimbChain.size());
  int n = SwingLimbChain.size();

  // Cost function on the norm difference between the reference avg position and the modified contact position.
  int neF = 1;
  neF = neF + NonlinearOptimizerInfo::RobotLinkInfo[_SwingLimbIndex].LocalContacts.size();     // The only constraint is for the contact to be non-penetrated.
  TransientOptProblem.InnerVariableInitialize(n, neF);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> xlow_vec(n), xupp_vec(n);
  for (int i = 0; i < n; i++)
  {
    // Configuration
    xlow_vec[i] = SimRobot.qMin(SwingLimbChain[i]);
    xupp_vec[i] = SimRobot.qMax(SwingLimbChain[i]);
    InitSwingLimbChain[i] = RefConfig[SwingLimbChain[i]];
  }
  TransientOptProblem.VariableBoundsUpdate(xlow_vec, xupp_vec);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> Flow_vec(neF), Fupp_vec(neF);
  for (int i = 0; i < neF; i++)
  {
    Flow_vec[i] = 0;
    Fupp_vec[i] = 1e10;
  }
  TransientOptProblem.ConstraintBoundsUpdate(Flow_vec, Fupp_vec);

  /*
    Initialize the seed guess
  */
  TransientOptProblem.SeedGuessUpdate(InitSwingLimbChain);

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
  TransientOptProblem.Solve(InitSwingLimbChain);

  std::vector<double> OptConfig = RefConfig;

  for (int i = 0; i < n; i++)
  {
    OptConfig[SwingLimbChain[i]] = InitSwingLimbChain[i];
  }
  SimRobotObj.UpdateConfig(Config(OptConfig));
  SimRobotObj.UpdateGeometry();
  bool SelfCollisionTest = SimRobotObj.SelfCollision();        // Self-collision has been included.
  switch(SelfCollisionTest)
  {
    case true:
    {
      OptFlag = false;
    }
    break;
    default:
    break;
  }
  double Pi = atan(1.0)*4.0;
  if(OptConfig[5]>Pi)
  {
    OptConfig[5]-=2.0 * Pi;
  }
  if(OptConfig[5]<-Pi)
  {
    OptConfig[5]+=2.0 * Pi;
  }
  return OptConfig;
}
