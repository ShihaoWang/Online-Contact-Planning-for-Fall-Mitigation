#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include <omp.h>
#include "NonlinearOptimizerInfo.h"

static Robot SimRobotObj;
static int SwingLimbIndex;
static std::vector<int> SwingLimbChain;
static Vector3 GradGoal;
static std::vector<double> RefConfig;
static std::vector<double> InitConfig;
static double EndEffectorProjTol;

struct EndEffectorOriOpt: public NonlinearOptimizerInfo
{
  EndEffectorOriOpt():NonlinearOptimizerInfo(){};

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
      std::vector<double> F_val = EndEffectorOriOptNCons(*n, *neF, x_vec);
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
  static std::vector<double> EndEffectorOriOptNCons(const int & nVar, const int & nObjNCons, const std::vector<double> & ActiveConfigOpt)
  {
    // This funciton provides the constraint for the configuration variable
    std::vector<double> F(nObjNCons);
    double AngDiff = 0.0;
    for (int i = 0; i < 2; i++)
    {
      RefConfig[SwingLimbChain[i]] = ActiveConfigOpt[i];
      double AngDiff_i = InitConfig[SwingLimbChain[i]] - ActiveConfigOpt[i];
      AngDiff+=AngDiff_i * AngDiff_i;
    }
    SimRobotObj.UpdateConfig(Config(RefConfig));
    SimRobotObj.UpdateGeometry();

    int ConstraintIndex = 0;
    F[ConstraintIndex] = AngDiff;
    ConstraintIndex+=1;

    RobotLink3D Link_i = SimRobotObj.links[NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LinkIndex];
    Vector3 AlignDirection;
    AlignDirection.x = Link_i.T_World.R.data[2][0];
    AlignDirection.y = Link_i.T_World.R.data[2][1];
    AlignDirection.z = Link_i.T_World.R.data[2][2];
    double Proj = AlignDirection.dot(GradGoal);
    F[ConstraintIndex] = Proj - EndEffectorProjTol;
    ConstraintIndex+=1;

    return F;
  }
};

std::vector<double> EndEffectorOriOptFn(const Robot & SimRobot, const std::vector<double> & _RefConfig, const int & _SwingLimbIndex, const Vector3 & _NormGoal, ReachabilityMap & RMObject, bool & OptFlag, const double & _EndEffectorProjTol)
{
  // This function is used to optimize robot's orienration.
  SimRobotObj = SimRobot;
  SwingLimbIndex = _SwingLimbIndex;
  SwingLimbChain = RMObject.EndEffectorLink2Pivotal[_SwingLimbIndex];
  RefConfig = _RefConfig;
  InitConfig = _RefConfig;
  GradGoal = _NormGoal;
  OptFlag = true;
  EndEffectorProjTol = _EndEffectorProjTol;

  EndEffectorOriOpt EndEffectorOriOptProblem;

  // Static Variable Substitution
  int n = 2;        // Only 2 is enough
  std::vector<double> InitSwingLimbChain(n);

  // Cost function on the norm difference between the reference avg position and the modified contact position.
  int neF = 1;
  neF += 1;
  EndEffectorOriOptProblem.InnerVariableInitialize(n, neF);

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
  EndEffectorOriOptProblem.VariableBoundsUpdate(xlow_vec, xupp_vec);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> Flow_vec(neF), Fupp_vec(neF);
  for (int i = 0; i < neF; i++)
  {
    Flow_vec[i] = 0;
    Fupp_vec[i] = 1e10;
  }
  EndEffectorOriOptProblem.ConstraintBoundsUpdate(Flow_vec, Fupp_vec);

  /*
    Initialize the seed guess
  */
  EndEffectorOriOptProblem.SeedGuessUpdate(InitSwingLimbChain);

  /*
    Given a name of this problem for the output
  */
  EndEffectorOriOptProblem.ProblemNameUpdate("EndEffectorOriOptProblem", 0);

  // Here we would like allow much more time to be spent on IK
  EndEffectorOriOptProblem.NonlinearProb.setIntParameter("Iterations limit", 5000);
  EndEffectorOriOptProblem.NonlinearProb.setIntParameter("Major iterations limit", 100);
  EndEffectorOriOptProblem.NonlinearProb.setIntParameter("Major print level", 0);
  EndEffectorOriOptProblem.NonlinearProb.setIntParameter("Minor print level", 0);
  /*
    ProblemOptions seting
  */
  // Solve with Finite-Difference
  EndEffectorOriOptProblem.ProblemOptionsUpdate(0, 3);
  EndEffectorOriOptProblem.Solve(InitSwingLimbChain);

  std::vector<double> OptConfig = RefConfig;

  for (int i = 0; i < n; i++)
  {
    OptConfig[SwingLimbChain[i]] = InitSwingLimbChain[i];
  }
  SimRobotObj.UpdateConfig(Config(OptConfig));
  SimRobotObj.UpdateGeometry();

  std::string ConfigPath = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/user/hrp2/";
  string _OptConfigFile = "EndEffectorOri.config";
  RobotConfigWriter(OptConfig, ConfigPath, _OptConfigFile);

  RobotLink3D Link_i = SimRobotObj.links[NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LinkIndex];
  Vector3 AlignDirection;
  AlignDirection.x = Link_i.T_World.R.data[2][0];
  AlignDirection.y = Link_i.T_World.R.data[2][1];
  AlignDirection.z = Link_i.T_World.R.data[2][2];
  double Proj = AlignDirection.dot(GradGoal);
  if(Proj>=EndEffectorProjTol) OptFlag = true;
  else OptFlag =false;
  OptConfig = YPRShifter(OptConfig);
  return OptConfig;
}
