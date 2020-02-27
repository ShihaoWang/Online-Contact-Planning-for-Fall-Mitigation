#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include <omp.h>
#include "NonlinearOptimizerInfo.h"

static Robot SimRobotObj;
static std::vector<int> ActiveJointIndices;
static std::vector<double> RefqOpt;
static Vector3 RefContact;
static Vector3 RefGrad;
static int SwingLimbIndex;
static SelfLinkGeoInfo SelfLinkGeoObj;

struct LastControlOpt: public NonlinearOptimizerInfo
{
  LastControlOpt():NonlinearOptimizerInfo(){};

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
      std::vector<double> F_val = LastControlOptNCons(*n, *neF, x_vec);
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
  static std::vector<double> LastControlOptNCons(const int & nVar, const int & nObjNCons, const std::vector<double> & ActiveConfigOpt)
  {
    // This funciton provides the constraint for the configuration variable
    std::vector<double> F(nObjNCons);
    for (int i = 0; i < ActiveJointIndices.size(); i++)
    {
      RefqOpt[ActiveJointIndices[i]] = ActiveConfigOpt[i];
    }
    SimRobotObj.UpdateConfig(Config(RefqOpt));
    Vector3 LinkiCenterPos;
    SimRobotObj.GetWorldPosition(RobotLinkInfo[SwingLimbIndex].AvgLocalContact, RobotLinkInfo[SwingLimbIndex].LinkIndex, LinkiCenterPos);
    Vector3 AvgDiff = LinkiCenterPos - RefContact;
    F[0] = AvgDiff.normSquared();

    int ConstraintIndex = 1;
    // Self-collision constraint
    std::vector<double> SelfCollisionDistVec(ActiveJointIndices.size());
    for (int i = 0; i < ActiveJointIndices.size(); i++)     // Due to the bounding box size of torso link
    {
      // Active Limb Center of Mass Position
      Vector3 JointiCenterPos;
      SimRobotObj.GetWorldPosition(Vector3(0.0, 0.0, 0.0), ActiveJointIndices[i], JointiCenterPos);
      double SignedDist;
      Vector3 SignedGrad;
      SelfLinkGeoObj.SelfCollisionDistNGrad(SwingLimbIndex, JointiCenterPos, SignedDist, SignedGrad);
      SelfCollisionDistVec[i] = SignedDist;
    }
    F[ConstraintIndex] = *std::min_element(SelfCollisionDistVec.begin(), SelfCollisionDistVec.end());
    ConstraintIndex+=1;

    RobotLink3D Link_i = SimRobotObj.links[RobotLinkInfo[SwingLimbIndex].LinkIndex];
    double norm_x = Link_i.T_World.R.data[2][0];
    double norm_y = Link_i.T_World.R.data[2][1];
    double norm_z = Link_i.T_World.R.data[2][2];
    Vector3 EndEffectorNorm(norm_x, norm_y, norm_z);

    double ProjNorm = RefGrad.dot(EndEffectorNorm);

    F[ConstraintIndex] = ProjNorm - 1.0;;
    ConstraintIndex+=1;

    return F;
  }
};

static std::vector<double> LastControlOptFn(const Robot & SimRobot, const std::vector<int> & ActiveJoints, const std::vector<double> & qOpt)
{
  // This function is used to optimize robot's configuration such that a certain contact can be reached for that end effector.
  SimRobotObj = SimRobot;
  LastControlOpt LastControlOptProblem;
  // Static Variable Substitution
  int n = ActiveJoints.size();

  // Cost function on the norm difference between the reference avg position and the modified contact position.
  int neF = 1;
  neF += 1;                                                                                       // Self-Collision
  // neF += 1;                                                                                       // Signed Distance
  neF += 1;                                                                                       // Surface Normal
  LastControlOptProblem.InnerVariableInitialize(n, neF);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> xlow_vec(n), xupp_vec(n);
  for (int i = 0; i < n; i++)
  {
    // Configuration
    xlow_vec[i] = SimRobot.qMin(ActiveJoints[i]);
    xupp_vec[i] = SimRobot.qMax(ActiveJoints[i]);
  }
  LastControlOptProblem.VariableBoundsUpdate(xlow_vec, xupp_vec);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> Flow_vec(neF), Fupp_vec(neF);
  for (int i = 0; i < neF; i++)
  {
    Flow_vec[i] = 0;
    Fupp_vec[i] = 1e10;
  }
  Flow_vec[neF-1] = 0.0;
  Fupp_vec[neF-1] = 0.0;
  LastControlOptProblem.ConstraintBoundsUpdate(Flow_vec, Fupp_vec);

  /*
    Initialize the seed guess
  */
  std::vector<double> qOptAct(ActiveJoints.size());
  for (int i = 0; i < ActiveJoints.size(); i++)
  {
    qOptAct[i] = qOpt[ActiveJoints[i]];
  }

  LastControlOptProblem.SeedGuessUpdate(qOptAct);

  /*
    Given a name of this problem for the output
  */
  LastControlOptProblem.ProblemNameUpdate("LastControlOptProblem", 0);

  // Here we would like allow much more time to be spent on IK
  LastControlOptProblem.NonlinearProb.setIntParameter("Iterations limit", 100);
  LastControlOptProblem.NonlinearProb.setIntParameter("Major iterations limit", 15);
  LastControlOptProblem.NonlinearProb.setIntParameter("Major print level", 0);
  LastControlOptProblem.NonlinearProb.setIntParameter("Minor print level", 0);
  /*
    ProblemOptions seting
  */
  // Solve with Finite-Difference
  LastControlOptProblem.ProblemOptionsUpdate(0, 3);
  LastControlOptProblem.Solve(qOptAct);

  std::vector<double> OptConfig = qOpt;

  for (int i = 0; i < n; i++)
  {
    OptConfig[ActiveJoints[i]] = qOptAct[i];
  }
  SimRobotObj.UpdateConfig(Config(OptConfig));
  SimRobotObj.UpdateGeometry();

  std::string ConfigPath = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/user/hrp2/";
  string _OptConfigFile = "LastContactOptConfig.config";
  RobotConfigWriter(OptConfig, ConfigPath, _OptConfigFile);

  bool SelfCollisionTest = SimRobotObj.SelfCollision();        // Self-collision has been included.
  if(SelfCollisionTest)
  {
    std::printf("Last Contact Optimization Failure due to Self-collision for Link %d! \n", NonlinearOptimizerInfo::RobotLinkInfo[SwingLimbIndex].LinkIndex);
    return YPRShifter(qOpt);
  }
  else
  {
    return YPRShifter(OptConfig);
  }
  return OptConfig;
}

std::vector<double> LastControlReference(const Robot & _SimRobot, const double & InitTime, const double & CurTime, ControlReferenceInfo & ControlReferenceObj, SelfLinkGeoInfo & _SelfLinkGeoObj, ReachabilityMap & RMObject)
{
  // This function is used to calculate robot's control for the last step.
  // We directly make use of robot's configuration for the underactuated part and get angles for the upper limb part but optimizing the lower limb part.
  Robot SimRobot = _SimRobot;
  std::vector<double> qReal = SimRobot.q;
  std::vector<double> qPlan = ControlReferenceObj.ConfigReference(InitTime, CurTime);
  std::vector<double> qOpt = qReal;
  SimRobot.UpdateConfig(Config(qPlan));     // To get robot's current end effector reference position.
  SimRobot.GetWorldPosition(NonlinearOptimizerInfo::RobotLinkInfo[ControlReferenceObj.SwingLimbIndex].AvgLocalContact, NonlinearOptimizerInfo::RobotLinkInfo[ControlReferenceObj.SwingLimbIndex].LinkIndex, RefContact);
  RefGrad = ControlReferenceObj.GoalContactGrad;
  std::vector<int> ActJoints = RMObject.EndEffectorLink2Pivotal[ControlReferenceObj.SwingLimbIndex];
  const int LastControlDOF = 3;       // Let's optimize end of end effector
  for (int i = 0; i < ActJoints.size(); i++)
  {
    qOpt[ActJoints[i]] = qPlan[ActJoints[i]];
  }
  RefqOpt = qOpt;
  std::vector<int> ActiveJoints(ActJoints.begin(), ActJoints.begin() + LastControlDOF);
  ActiveJointIndices = ActiveJoints;
  SwingLimbIndex = ControlReferenceObj.SwingLimbIndex;
  SelfLinkGeoObj = _SelfLinkGeoObj;
  return qOpt = LastControlOptFn(SimRobot, ActiveJoints, qOpt);
}
