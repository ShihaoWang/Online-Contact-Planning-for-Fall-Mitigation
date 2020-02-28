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
static Vector3 GradGoal;
static std::vector<double> RefConfig;
static double PosGoalDist;
static SelfLinkGeoInfo SelfLinkGeoObj;
static double SelfCollisionTol = 0.01;

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
    Vector3 LinkiCenterPos;
    SimRobotObj.GetWorldPosition(NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].AvgLocalContact, NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LinkIndex, LinkiCenterPos);
    Vector3 AvgDiff = LinkiCenterPos - PosGoal;
    F[0] = AvgDiff.normSquared();

    int ConstraintIndex = 1;
    for (int i = 0; i < NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LocalContacts.size(); i++)
    {
      Vector3 LinkiPjPos;
      SimRobotObj.GetWorldPosition(NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LocalContacts[i], NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LinkIndex, LinkiPjPos);
      F[ConstraintIndex] = SDFInfo.SignedDistance(LinkiPjPos) - PosGoalDist;
      ConstraintIndex+=1;
    }

    // Self-collision constraint
    std::vector<double> SelfCollisionDistVec(SwingLimbChain.size()-3);
    for (int i = 0; i < SwingLimbChain.size()-3; i++)     // Due to the bounding box size of torso link
    {
      // Active Limb Center of Mass Position
      Vector3 JointiCenterPos;
      SimRobotObj.GetWorldPosition(Vector3(0.0, 0.0, 0.0), SwingLimbChain[i], JointiCenterPos);
      double SignedDist;
      Vector3 SignedGrad;
      SelfLinkGeoObj.SelfCollisionDistNGrad(SwingLimbIndex, JointiCenterPos, SignedDist, SignedGrad);
      SelfCollisionDistVec[i] = SignedDist;
      std::vector<Vector3> Vertices = SelfLinkGeoObj.BBVertices(SwingLimbChain[i]-5);
    }

    F[ConstraintIndex] = *std::min_element(SelfCollisionDistVec.begin(), SelfCollisionDistVec.end()) - SelfCollisionTol;
    ConstraintIndex+=1;

    F[ConstraintIndex] = SDFInfo.SignedDistance(LinkiCenterPos);
    ConstraintIndex+=1;

    return F;
  }
};

std::vector<double> TransientOptFn(const Robot & SimRobot, const int & _SwingLimbIndex, SelfLinkGeoInfo & _SelfLinkGeoObj, const Vector3 & _PosGoal, ReachabilityMap & RMObject, bool & OptFlag, const bool & LastFlag)
{
  // This function is used to optimize robot's configuration such that a certain contact can be reached for that end effector.
  SimRobotObj = SimRobot;
  SwingLimbIndex = _SwingLimbIndex;
  SwingLimbChain = RMObject.EndEffectorLink2Pivotal[_SwingLimbIndex];
  PosGoal = _PosGoal;
  PosGoalDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(PosGoal);
  PosGoalDist = max(0.0, PosGoalDist);
  RefConfig = SimRobot.q;
  SelfLinkGeoObj = _SelfLinkGeoObj;
  OptFlag = true;

  TransientOpt TransientOptProblem;

  // Static Variable Substitution
  std::vector<double> InitSwingLimbChain(SwingLimbChain.size());
  int n = SwingLimbChain.size();

  // Cost function on the norm difference between the reference avg position and the modified contact position.
  int neF = 1;
  neF = neF + NonlinearOptimizerInfo::RobotLinkInfo[_SwingLimbIndex].LocalContacts.size();        // The only constraint is for the contact to be non-penetrated.
  neF += 1;                                                                                       // Self-Collision
  neF += 1;                                                                                       // Signed Distance
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
  if(LastFlag)
  {
    Flow_vec[neF-1] = 0;
    Fupp_vec[neF-1] = 0;
  }
  GradGoal = NonlinearOptimizerInfo::SDFInfo.SignedDistanceNormal(PosGoal);
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

  std::string ConfigPath = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/user/hrp2/";
  string _OptConfigFile = "InnerOptConfig.config";
  RobotConfigWriter(OptConfig, ConfigPath, _OptConfigFile);

  bool SelfCollisionTest = SimRobotObj.SelfCollision();        // Self-collision has been included.
  if(SelfCollisionTest)
  {
    std::printf("Transient Optimization Failure due to Self-collision for Link %d! \n", NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LinkIndex);
    OptFlag = false;
  }
  if(LastFlag)
  {
    // Then check signed distance of end effector!
    Vector3 LinkiCenterPos;
    SimRobotObj.GetWorldPosition(NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].AvgLocalContact, NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LinkIndex, LinkiCenterPos);
    double EndEffectorDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(LinkiCenterPos);
    if(EndEffectorDist>0.01)
    {
      std::printf("Transient Optimization Failure due to Goal Contact Non-reachability for Link %d! \n", NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LinkIndex);
      OptFlag = false;
    }
  }
  OptConfig = YPRShifter(OptConfig);
  return OptConfig;
}
