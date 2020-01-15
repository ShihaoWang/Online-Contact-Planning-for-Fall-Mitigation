#include "NonlinearOptimizerInfo.h"
#include "CommonHeader.h"

static Robot SimRobotObj;
static std::vector<ContactStatusInfo> RobotContactInfo;
static std::vector<double> RobotConfigRef;

struct InitialConfigurationOpt: public NonlinearOptimizerInfo
{
  // This is the class struct for the optimizaiton of configuraiton variables

  InitialConfigurationOpt():NonlinearOptimizerInfo(){};

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
      std::vector<double> F_val = InitialConfigObjNCons(*n, *neF, x_vec);
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
  static std::vector<double> InitialConfigObjNCons(const int & nVar, const int & nObjNCons, const std::vector<double> & ConfigOpt)
  {
    // This funciton provides the constraint for the configuration variable

    std::vector<double> F(nObjNCons);
    Config ConfigOptNew(ConfigOpt);
    SimRobotObj.UpdateConfig(ConfigOptNew);     // Here both the SimRobot.q and robot frames have already been updated.
    double ConfigVia = 0.0;
    for (int i = 0; i <nVar; i++)
    // for (int i = 0; i <6; i++)
    {
      ConfigVia +=(ConfigOpt[i] - RobotConfigRef[i]) * (ConfigOpt[i] - RobotConfigRef[i]);
    }
    F[0] =  ConfigVia;
    // Make sure that active end effectors have zero relative signed distance.
    int ConstraintIndex = 1;
    for (int i = 0; i < RobotLinkInfo.size(); i++)
    {
      for (int j = 0; j < RobotLinkInfo[i].LocalContacts.size(); j++)
      {
        Vector3 LinkiPjPos;
        SimRobotObj.GetWorldPosition(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, LinkiPjPos);
        F[ConstraintIndex] = NonlinearOptimizerInfo::SDFInfo.SignedDistance(LinkiPjPos);      ConstraintIndex = ConstraintIndex + 1;
      }
    }

    // The last constraint is that all the links have to remain nonnegative with respect to the environment.
    // The robot link COM position
    for (int i = 0; i < nVar-6; i++)
    {
      Vector3 Link_i_COM;
      SimRobotObj.links[i+6].GetWorldCOM(Link_i_COM);
      F[ConstraintIndex] = NonlinearOptimizerInfo::SDFInfo.SignedDistance(Link_i_COM);     // Here 0.001 is the epsilon addition
      ConstraintIndex = ConstraintIndex + 1;
    }

    // The last constraint is to make sure that Center of Mass remains to be within SP
    std::vector<Vector3> SPVertices;
    for (int i = 0; i < RobotLinkInfo.size(); i++)
    {
      int LinkiPNo = RobotLinkInfo[i].LocalContacts.size();
      for (int j = 0; j < LinkiPNo; j++)
      {
        switch (RobotContactInfo[i].LocalContactStatus[j])
        {
          case 0:
          break;
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
    F[ConstraintIndex] = SPObj.ProjPoint2EdgeDist(COM_Pos) - 0.025;
    ConstraintIndex = ConstraintIndex + 1;

    return F;
  }
};

static std::vector<double> InitialConfigOptFn(std::vector<double> &RobotConfig)
{
  // This function is used to optimize the robot configuraiton variable such that certain contact is active.
  InitialConfigurationOpt InitialConfigOptProblem;

  // Static Variable Substitution
  int n = SimRobotObj.q.size();
  int neF = 1;                                                                  // Cost function on the norm difference between the reference configuration and the optimized configuration
  for (int i = 0; i < RobotContactInfo.size(); i++)
  {
    for (int j = 0; j < RobotContactInfo[i].LocalContactStatus.size(); j++)    // Each contact has a distance constraint.
    {
      neF = neF + 1;
    }
  }
  neF = neF + n - 6;
  neF = neF + 1;      // Add one more constraint on the CoM within SP
  InitialConfigOptProblem.InnerVariableInitialize(n, neF);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> xlow_vec(n), xupp_vec(n);

  for (int i = 0; i < n; i++)
  {
    // Configuration
    xlow_vec[i] = SimRobotObj.qMin(i);
    xupp_vec[i] = SimRobotObj.qMax(i);
  }
  InitialConfigOptProblem.VariableBoundsUpdate(xlow_vec, xupp_vec);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> Flow_vec(neF), Fupp_vec(neF);
  Flow_vec[0] = 0;
  Fupp_vec[0] = 1e20;           // The default idea is that the objective should always be nonnegative
  int ConstraintIndex = 1;
  for (int i = 0; i < RobotContactInfo.size(); i++)
  {
    for (int j = 0; j < RobotContactInfo[i].LocalContactStatus.size(); j++)
    {
      // Distance Constraint: scalar
      switch (RobotContactInfo[i].LocalContactStatus[j])
      {
        case 1:     // This means that the certain constraint is active
        Flow_vec[ConstraintIndex] = 0;
        Fupp_vec[ConstraintIndex] = 0;
        ConstraintIndex = ConstraintIndex + 1;
        break;
        case 0:
        Flow_vec[ConstraintIndex] = 0;
        Fupp_vec[ConstraintIndex] = 1e20;         // This is due to the nonpenetraition consideration.
        ConstraintIndex = ConstraintIndex + 1;
        break;
      }
    }
  }
  for (int i = 0; i < SimRobotObj.q.size()-6; i++)
  {
    Flow_vec[ConstraintIndex] = 0;
    Fupp_vec[ConstraintIndex] = 1e20;         // This is due to the nonpenetraition consideration.
    ConstraintIndex = ConstraintIndex + 1;
  }

  // Projected Center of Mass within Support Polygon
  Flow_vec[ConstraintIndex] = 0;
  Fupp_vec[ConstraintIndex] = 1e20;
  ConstraintIndex = ConstraintIndex + 1;

  InitialConfigOptProblem.ConstraintBoundsUpdate(Flow_vec, Fupp_vec);

  /*
    Initialize the seed guess
  */
  InitialConfigOptProblem.SeedGuessUpdate(RobotConfigRef);

  /*
    Given a name of this problem for the output
  */
  InitialConfigOptProblem.ProblemNameUpdate("InitialConfigOptProblem", 0);

  InitialConfigOptProblem.NonlinearProb.setIntParameter("Iterations limit", 1000000);
  InitialConfigOptProblem.NonlinearProb.setIntParameter("Major iterations limit", 250);
  InitialConfigOptProblem.NonlinearProb.setIntParameter("Major print level", 0);
  InitialConfigOptProblem.NonlinearProb.setIntParameter("Minor print level", 0);
  /*
    ProblemOptions seting
  */
  // Solve with Finite-Difference
  InitialConfigOptProblem.ProblemOptionsUpdate(0, 3);
  InitialConfigOptProblem.Solve(RobotConfig);

  return RobotConfig;
}

std::vector<double> InitialConfigurationOptimization(Robot& _SimRobotObj, const std::vector<ContactStatusInfo> &  _RobotContactInfo, const std::vector<double>& _RobotConfigRef)
{
  SimRobotObj = _SimRobotObj;
  RobotContactInfo = _RobotContactInfo;
  RobotConfigRef = _RobotConfigRef;

  std::vector<double> RobotConfig(_RobotConfigRef.size(), 0);
  return InitialConfigOptFn(RobotConfig);
}
