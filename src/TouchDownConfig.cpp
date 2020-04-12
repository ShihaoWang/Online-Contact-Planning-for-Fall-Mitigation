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
static std::vector<double> InitConfig;
static SelfLinkGeoInfo SelfLinkGeoObj;

struct TouchDownConfigOpt: public NonlinearOptimizerInfo
{
  TouchDownConfigOpt():NonlinearOptimizerInfo(){};

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
      std::vector<double> F_val = TouchDownConfigOptNCons(*n, *neF, x_vec);
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
  static std::vector<double> TouchDownConfigOptNCons(const int & nVar, const int & nObjNCons, const std::vector<double> & ActiveConfigOpt)
  {
    // This funciton provides the constraint for the configuration variable
    std::vector<double> F(nObjNCons);
    double ConfigDiff = 0.0;
    for (int i = 0; i < SwingLimbChain.size(); i++)
    {
      RefConfig[SwingLimbChain[i]] = ActiveConfigOpt[i];
      double ConfigDiff_i = InitConfig[SwingLimbChain[i]] - ActiveConfigOpt[i];
      ConfigDiff+=ConfigDiff_i*ConfigDiff_i;
    }
    SimRobotObj.UpdateConfig(Config(RefConfig));
    SimRobotObj.UpdateGeometry();
    F[0] = ConfigDiff;

    int ConstraintIndex = 1;
    // Self-collision constraint
    std::vector<double> SelfCollisionDistVec(SwingLimbChain.size()-3);
    for (int i = 0; i < SwingLimbChain.size()-3; i++)     // Due to the bounding box size of torso link
    {
      Box3D Box3DObj = SimRobotObj.geometry[SwingLimbChain[i]]->GetBB();
      std::vector<Vector3> BoxVerticesVec = BoxVertices(Box3DObj);
      std::vector<double> DistVec(BoxVerticesVec.size());
      for (int j = 0; j < BoxVerticesVec.size(); j++)
      {
        DistVec[j] = SelfLinkGeoObj.SelfCollisionDist(SwingLimbIndex, BoxVerticesVec[j]);
      }
      SelfCollisionDistVec[i] = *std::min_element(DistVec.begin(), DistVec.end());
    }

    F[ConstraintIndex] = *std::min_element(SelfCollisionDistVec.begin(), SelfCollisionDistVec.end());
    ConstraintIndex+=1;

    for (int i = 0; i < NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LocalContacts.size(); i++)
    {
      Vector3 LinkiPjPos;
      SimRobotObj.GetWorldPosition(NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LocalContacts[i], NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LinkIndex, LinkiPjPos);
      F[ConstraintIndex] = SDFInfo.SignedDistance(LinkiPjPos);
      ConstraintIndex+=1;
    }
    return F;
  }
};

std::vector<double> TouchDownConfigOptFn(const Robot & SimRobot, const int & _SwingLimbIndex, SelfLinkGeoInfo & _SelfLinkGeoObj, ReachabilityMap & RMObject, bool & OptFlag)
{
  // This function is used to optimize robot's touch down configuration such that the end effector touches the environment without self-collision.
  SimRobotObj = SimRobot;
  SwingLimbIndex = _SwingLimbIndex;
  SwingLimbChain = RMObject.EndEffectorLink2Pivotal[_SwingLimbIndex];
  RefConfig = SimRobot.q;
  SelfLinkGeoObj = _SelfLinkGeoObj;
  OptFlag = true;

  TouchDownConfigOpt TouchDownConfigOptProblem;

  // Static Variable Substitution
  std::vector<double> InitSwingLimbChain(SwingLimbChain.size());
  int n = SwingLimbChain.size();

  // Cost function on the norm difference between the reference avg position and the modified contact position.
  int neF = 1;
  neF += 1;                                                                                       // Self-Collision
  neF = neF + NonlinearOptimizerInfo::RobotLinkInfo[_SwingLimbIndex].LocalContacts.size();        // Touch-down constraint
  TouchDownConfigOptProblem.InnerVariableInitialize(n, neF);

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
  TouchDownConfigOptProblem.VariableBoundsUpdate(xlow_vec, xupp_vec);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> Flow_vec(neF), Fupp_vec(neF);
  for (int i = 0; i < 2; i++)
  {
    Flow_vec[i] = 0;
    Fupp_vec[i] = 1e10;
  }
  for (int i = 2; i < neF; i++)
  {
    Flow_vec[i] = 0;
    Fupp_vec[i] = 0;
  }
  TouchDownConfigOptProblem.ConstraintBoundsUpdate(Flow_vec, Fupp_vec);

  /*
    Initialize the seed guess
  */
  TouchDownConfigOptProblem.SeedGuessUpdate(InitSwingLimbChain);

  /*
    Given a name of this problem for the output
  */
  TouchDownConfigOptProblem.ProblemNameUpdate("TouchDownConfigOptProblem", 0);

  // Here we would like allow much more time to be spent on IK
  TouchDownConfigOptProblem.NonlinearProb.setIntParameter("Iterations limit", 250);
  TouchDownConfigOptProblem.NonlinearProb.setIntParameter("Major iterations limit", 25);
  TouchDownConfigOptProblem.NonlinearProb.setIntParameter("Major print level", 0);
  TouchDownConfigOptProblem.NonlinearProb.setIntParameter("Minor print level", 0);
  /*
    ProblemOptions seting
  */
  // Solve with Finite-Difference
  TouchDownConfigOptProblem.ProblemOptionsUpdate(0, 3);
  TouchDownConfigOptProblem.Solve(InitSwingLimbChain);

  std::vector<double> OptConfig = RefConfig;

  for (int i = 0; i < n; i++)
  {
    OptConfig[SwingLimbChain[i]] = InitSwingLimbChain[i];
  }
  SimRobotObj.UpdateConfig(Config(OptConfig));
  SimRobotObj.UpdateGeometry();

  std::string ConfigPath = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/user/hrp2/";
  string _OptConfigFile = "TouchDownConfig.config";
  RobotConfigWriter(OptConfig, ConfigPath, _OptConfigFile);

  // Self-collision constraint numerical checker
  std::vector<double> SelfCollisionDistVec(SwingLimbChain.size()-3);
  for (int i = 0; i < SwingLimbChain.size()-3; i++)     // Due to the bounding box size of torso link
  {
    Box3D Box3DObj = SimRobotObj.geometry[SwingLimbChain[i]]->GetBB();
    std::vector<Vector3> BoxVerticesVec = BoxVertices(Box3DObj);
    Vector3Writer(BoxVerticesVec, "BoxPoints");
    std::vector<double> DistVec(BoxVerticesVec.size());
    for (int j = 0; j < BoxVerticesVec.size(); j++)
    {
      DistVec[j] = SelfLinkGeoObj.SelfCollisionDist(SwingLimbIndex, BoxVerticesVec[j]);
    }
    SelfCollisionDistVec[i] = *std::min_element(DistVec.begin(), DistVec.end());
  }
  double SelfCollisionDistTol = *std::min_element(SelfCollisionDistVec.begin(), SelfCollisionDistVec.end());

  if(SelfCollisionDistTol<-0.0025){
      std::printf("Transient Optimization Failure due to Self-collision for Link %d! \n", NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LinkIndex);
      OptFlag = false;
  }

  Vector3 LinkiCenterPos;
  SimRobotObj.GetWorldPosition(NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].AvgLocalContact, NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LinkIndex, LinkiCenterPos);
  Vector3 AvgDiff = LinkiCenterPos - PosGoal;
  double DistTestTol = 0.0225;
  double DistTest = AvgDiff.normSquared();
  if(DistTest>DistTestTol){
    std::printf("Transient Optimization Failure due to Goal Contact Non-reachability for Link %d! \n", NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LinkIndex);
    OptFlag = false;
  }

  OptConfig = YPRShifter(OptConfig);
  return OptConfig;
}
