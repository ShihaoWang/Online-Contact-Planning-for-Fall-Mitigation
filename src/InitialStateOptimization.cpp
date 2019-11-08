#include "NonlinearOptimizerInfo.h"
#include "CommonHeader.h"

static Robot SimRobotObj;
static std::vector<LinkInfo> RobotLinkInfo;
static std::vector<ContactStatusInfo> RobotContactInfo;
static SignedDistanceFieldInfo SDFInfo;
static std::vector<double> RobotConfigRef;

static double KEInit;
static Vector3 CentDirection;
static double eps = 1e-8;

struct InitialConfigOpt: public NonlinearOptimizerInfo
{
  // This is the class struct for the optimizaiton of configuraiton variables

  InitialConfigOpt():NonlinearOptimizerInfo(){};

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
  static std::vector<double> InitialConfigObjNCons(const int&nVar, const int &nObjNCons, const std::vector<double>& ConfigOpt)
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
        F[ConstraintIndex] = SDFInfo.SignedDistance(LinkiPjPos);      ConstraintIndex = ConstraintIndex + 1;
      }
    }

    // The last constraint is that all the links have to remain nonnegative with respect to the environment.
    // The robot link COM position
    for (int i = 0; i < nVar-6; i++)
    {
      Vector3 Link_i_COM;
      SimRobotObj.links[i+6].GetWorldCOM(Link_i_COM);
      F[ConstraintIndex] = SDFInfo.SignedDistance(Link_i_COM);     // Here 0.001 is the epsilon addition
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

static void InitialConfigOptFn(std::vector<double> &RobotConfig)
{
  // This function is used to optimize the robot configuraiton variable such that certain contact is active.
  InitialConfigOpt InitialConfigOptProblem;

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
  return;
}

struct InitialVelocityOpt: public NonlinearOptimizerInfo
{
  // This is the class struct for the optimizaiton of configuraiton variables

  InitialVelocityOpt():NonlinearOptimizerInfo(){};

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
      std::vector<double> F_val = InitialVelocityObjNCons(*n, *neF, x_vec);
      for (int i = 0; i < *neF; i++)
      {
        F[i] = F_val[i];
      }
    }
  void Solve(std::vector<double> &RobotVelocity, double & SlackVariable)
  {
    int StartType = 0;
    NonlinearProb.solve(StartType, neF, n, ObjAdd, ObjRow, ObjNConstraint,
      xlow, xupp, Flow, Fupp,
      x, xstate, xmul, F, Fstate, Fmul,
      nS, nInf, sumInf);
      for (int i = 0; i < n-1; i++)
      {
        RobotVelocity[i] = x[i];
      }
      SlackVariable = x[n-1];
      delete []x;      delete []xlow;   delete []xupp;
      delete []xmul;   delete []xstate;

      delete []F;      delete []Flow;   delete []Fupp;
      delete []Fmul;   delete []Fstate;
  }

  static std::vector<double> InitialVelocityObjNCons(const int& nVar, const int& nObjNCons, const std::vector<double>& VariableOpt)
  {
    // This funciton provides the constraint for the configuration variable
    std::vector<double> F(nObjNCons);

    std::vector<double> VelocityOpt(SimRobotObj.dq.size());
    double Scale = VariableOpt[SimRobotObj.dq.size()];
    F[0] = 0.0;
    for (int i = 0; i < SimRobotObj.dq.size(); i++)
    {
      VelocityOpt[i] = VariableOpt[i];
      F[0]+= VelocityOpt[i] * VelocityOpt[i];
    }
    SimRobotObj.dq = VelocityOpt;
    // Make sure that active end effectors have zero relative signed distance.
    int ConstraintIndex = 1;
    // According to the RobotContactInfo, certain contacts are not active
    for (int i = 0; i < RobotLinkInfo.size(); i++)
    {
      for (int j = 0; j < RobotLinkInfo[i].LocalContacts.size(); j++)
      {
        Vector3 LinkiPjVel;
        SimRobotObj.GetWorldVelocity(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, SimRobotObj.dq, LinkiPjVel);
        F[ConstraintIndex] = LinkiPjVel.x;       ConstraintIndex = ConstraintIndex + 1;
        F[ConstraintIndex] = LinkiPjVel.y;       ConstraintIndex = ConstraintIndex + 1;
        F[ConstraintIndex] = LinkiPjVel.z;       ConstraintIndex = ConstraintIndex + 1;
      }
    }

    Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
    CentroidalState(SimRobotObj, COMPos, COMVel);

    // Here we here four constraints: kinetic energy, centroidal direction
    double Kinetic_Energy = SimRobotObj.GetKineticEnergy();
    F[ConstraintIndex] = Kinetic_Energy - KEInit;
    ConstraintIndex = ConstraintIndex + 1;

    F[ConstraintIndex] = COMVel.x - Scale * CentDirection.x;
    ConstraintIndex = ConstraintIndex + 1;

    F[ConstraintIndex] = COMVel.y - Scale * CentDirection.y;
    ConstraintIndex = ConstraintIndex + 1;

    F[ConstraintIndex] = COMVel.z - Scale * CentDirection.z;
    ConstraintIndex = ConstraintIndex + 1;
    return F;
  }
};

static bool InitialVelocityOptFn(const double & KEInit, const Vector3 & CentDirection, std::vector<double> & RobotVelocity)
{
  // This function is used to optimize the robot configuraiton variable such that certain contact is active.
  InitialVelocityOpt InitialVelocityOptProblem;

  int n = SimRobotObj.dq.size() + 1;                                            // DOF + slack variable
  int neF = 1;                                                                  // Cost function sum on the robot's velocity square
  for (int i = 0; i < RobotContactInfo.size(); i++)
  {
    for (int j = 0; j < RobotContactInfo[i].LocalContactStatus.size(); j++)     // Each contact has a distance constraint.
    {
      neF = neF + 3;
    }
  }
  neF = neF + 4;                                                                // Robot's Centroidal Direction
  InitialVelocityOptProblem.InnerVariableInitialize(n, neF);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> xlow_vec(n), xupp_vec(n);

  for (int i = 0; i < 6; i++)
  {
    // Bounds on the global variables
    xlow_vec[i] = -1000.0;
    xupp_vec[i] =  1000.0;
  }
  for (int i = 6; i < n-1; i++)
  {
    xlow_vec[i] = SimRobotObj.velMin(i);
    xupp_vec[i] = SimRobotObj.velMax(i);
  }
  // A positive lower and higher bound
  xlow_vec[n-1] = 0.001;      // 0.001
  xupp_vec[n-1] = 10.0;
  InitialVelocityOptProblem.VariableBoundsUpdate(xlow_vec, xupp_vec);

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
        Flow_vec[ConstraintIndex] = 0;
        Fupp_vec[ConstraintIndex] = 0;
        ConstraintIndex = ConstraintIndex + 1;
        Flow_vec[ConstraintIndex] = 0;
        Fupp_vec[ConstraintIndex] = 0;
        ConstraintIndex = ConstraintIndex + 1;
        break;
        case 0:
        Flow_vec[ConstraintIndex] = -1e20;
        Fupp_vec[ConstraintIndex] = 1e20;         // This is due to the nonpenetraition consideration.
        ConstraintIndex = ConstraintIndex + 1;
        Flow_vec[ConstraintIndex] = -1e20;
        Fupp_vec[ConstraintIndex] = 1e20;         // This is due to the nonpenetraition consideration.
        ConstraintIndex = ConstraintIndex + 1;
        Flow_vec[ConstraintIndex] = -1e20;
        Fupp_vec[ConstraintIndex] = 1e20;         // This is due to the nonpenetraition consideration.
        ConstraintIndex = ConstraintIndex + 1;
        break;
      }
    }
  }
  Flow_vec[ConstraintIndex] = 0.0;
  Fupp_vec[ConstraintIndex] = 0.0;                // Robot's Centroidal Position x
  ConstraintIndex = ConstraintIndex + 1;
  Flow_vec[ConstraintIndex] = 0.0;
  Fupp_vec[ConstraintIndex] = 0.0;                // Robot's Centroidal Position y
  ConstraintIndex = ConstraintIndex + 1;
  Flow_vec[ConstraintIndex] = 0.0;
  Fupp_vec[ConstraintIndex] = 0.0;                // Robot's Centroidal Position z
  ConstraintIndex = ConstraintIndex + 1;
  Flow_vec[ConstraintIndex] = 0.0;
  Fupp_vec[ConstraintIndex] = 0.0;                // Robot's Kinetic Energy
  ConstraintIndex = ConstraintIndex + 1;
  InitialVelocityOptProblem.ConstraintBoundsUpdate(Flow_vec, Fupp_vec);
  /*
    Initialize the seed guess
  */
  std::vector<double> GuessVec(n);
  for (int i = 0; i < n; i++)
  {
    GuessVec[i] = RobotVelocity[i];
  }
  InitialVelocityOptProblem.SeedGuessUpdate(GuessVec);

  /*
    Given a name of this problem for the output
  */
  InitialVelocityOptProblem.ProblemNameUpdate("InitialVelocityOptProblem", 0);

  InitialVelocityOptProblem.NonlinearProb.setIntParameter("Iterations limit", 1000000);
  InitialVelocityOptProblem.NonlinearProb.setIntParameter("Major iterations limit", 500);
  InitialVelocityOptProblem.NonlinearProb.setIntParameter("Major print level", 0);
  InitialVelocityOptProblem.NonlinearProb.setIntParameter("Minor print level", 0);
  /*
    ProblemOptions seting
  */
  // Solve with Finite-Difference
  InitialVelocityOptProblem.ProblemOptionsUpdate(0, 3);
  double Scale;
  InitialVelocityOptProblem.Solve(RobotVelocity, Scale);

  // The last step is the constraint validation.
  std::vector<double> VariableOpt(n);
  for (int i = 0; i < n-1; i++)
  {
    VariableOpt[i] = RobotVelocity[i];
  }
  VariableOpt[n-1] = Scale;
  std::vector<double> ObjNConstraintVal = InitialVelocityOptProblem.InitialVelocityObjNCons(n, neF, VariableOpt);

  double KEVio = ObjNConstraintVal[neF-4] * ObjNConstraintVal[neF-4];
  double CentxVio = ObjNConstraintVal[neF-3] * ObjNConstraintVal[neF-3];
  double CentyVio = ObjNConstraintVal[neF-2] * ObjNConstraintVal[neF-2];
  double CentzVio = ObjNConstraintVal[neF-1] * ObjNConstraintVal[neF-1];

  std::vector<double> Vio = {KEVio, CentxVio, CentyVio, CentzVio};
  double VioMax = *std::max_element(Vio.begin(), Vio.end());

  bool FeasibleFlag = true;
  if (VioMax>eps)
  {
    FeasibleFlag = false;
  }

  return FeasibleFlag;
}

bool InitialStateOptFn(Robot& _SimRobotObj, const std::vector<LinkInfo> & _RobotLinkInfo, const std::vector<ContactStatusInfo> &  _RobotContactInfo, const SignedDistanceFieldInfo& _SDFInfo, const std::vector<double>& _RobotConfigRef, const double & _KEInit, const Vector3& _CentDirection, std::vector<double> & RobotConfig, std::vector<double> & RobotVelocity, const bool & ConfigFlag, const bool & VelocityFlag)
{
  SimRobotObj = _SimRobotObj;
  RobotLinkInfo = _RobotLinkInfo;
  RobotContactInfo = _RobotContactInfo;
  SDFInfo = _SDFInfo;
  RobotConfigRef = _RobotConfigRef;

  KEInit = _KEInit;
  CentDirection = _CentDirection;

  switch (ConfigFlag)
  {
    case true:
    {
      InitialConfigOptFn(RobotConfig);
    }
    break;
    default:
    {
    }
    break;
  }
  RobotConfigRef = RobotConfig;
  Config RobotConfigNew(RobotConfig);
  SimRobotObj.UpdateConfig(RobotConfigNew);
  switch (VelocityFlag)
  {
    case true:
    {
      return InitialVelocityOptFn(KEInit, CentDirection, RobotVelocity);
    }
    break;
    default:
    {
      for (int i = 0; i < RobotVelocity.size(); i++)
      {
        RobotVelocity[i] = 0.0;
      }
    }
    break;
  }
  return true;
}
