#include "NonlinearOptimizerInfo.h"
#include "CommonHeader.h"
#include <random>
#include "gurobi_c++.h"

// This function is used to compute the stabilizing controller to regain balance for the disturbed robot.

static std::vector<Vector3> ConeAllUnits;
static std::vector<Matrix> ActJacobians;
static int EdgeNo;
static int DOF;
static std::vector<LinkInfo> RobotLinkInfo;
static std::vector<ContactStatusInfo> RobotContactInfo;
static std::vector<double> RobotConfigRef;

// QP Gains
static double Kpos = 0.0;
static double Kvel = 20.0;         // This gain should not be so big for stabilization.  1 works! 2 works! 5 kinda
static double Kqddot = 5.0;
static double Ku = 0.0;
static double Kf = 0.0;
static double Keta = 0.0;

static double ConAccTol = 0.05;

static double epsTol = 1e-8;

using namespace std;

static int ContactPointNo;

static double InfVal = 10000;

static void AccBndGene(const double & x, const double & v, const double & xLow, const double & xUpp, const double & vLimit, const double & aLimit, double & aLow, double & aUpp)
{
  // This function is used to generate robot's bound on configuration and velocity.
  // Robot's configuration and velocity have to be bounded within a valid range.

  double eps = 1e-6;

  double xToLeftBnd = (x - xLow) * (x - xLow);
  double xToRightBnd = (x - xUpp) * (x - xUpp);
  double vToDownBnd = (v + vLimit) * (v + vLimit);
  double vToUppBnd = (v - vLimit) * (v - vLimit);

  if(x>=0)
  {
    // In the right region of x-v plane
    if(xToRightBnd<=eps)
    {
      // At right bound
      aLow = 0.0;
      aUpp = 0.0;
      return;
    }
    else
    {
      if(vToDownBnd<=eps)
      {
        aLow = aLimit;
        aUpp = aLimit + eps;
        return;
      }
      if(vToUppBnd<=eps)
      {
        aLow = -aLimit;
        aUpp = -aLimit + eps;
        return;
      }
      // Region I comparison.
      double RegionI = x + 0.5 * v * v/aLimit;
      if(RegionI>=xUpp)
      {
        aLow = -aLimit;
        aUpp = -aLimit + eps;
      }
    }
  }
  else
  {
    if(vToUppBnd<=eps)
    {
      aLow = -aLimit;
      aUpp = -aLimit + eps;
      return;
    }
    if(xToLeftBnd<=eps)
    {
      aLow = 0.0;
      aUpp = 0.0;
      return;
    }
    double RegionII = x - 0.5 * v * v/aLimit;
    if(RegionII<=xLow)
    {
      aLow = aLimit;
      aUpp = aLimit + eps;
      return;
    }
    if(vToDownBnd<=eps)
    {
      aLow = aLimit;
      aUpp = aLimit + eps;
      return;
    }
  }

  aLow = -aLimit;
  aUpp = aLimit;
  return;
}


static void AccBndGenerator(const double & x, const double & v, const double & xLow, const double & xUpp, const double & vLimit, const double & aLimit, double & aLow, double & aUpp)
{
  // This function is used to generate robot's bound on configuration and velocity.
  // Robot's configuration and velocity have to be bounded within a valid range.

  double eps = 1e-6;

  double xToLeftBnd = (x - xLow) * (x - xLow);
  double xToRightBnd = (x - xUpp) * (x - xUpp);
  double vToDownBnd = (v + vLimit) * (v + vLimit);
  double vToUppBnd = (v - vLimit) * (v - vLimit);

  if(x>=0)
  {
    // In the right region of x-v plane
    if(xToRightBnd<=eps)
    {
      // At right bound
      aLow = -aLimit;
      aUpp = -aLimit + eps;
      return;
    }
    else
    {
      if(vToDownBnd<=eps)
      {
        aLow = aLimit;
        aUpp = aLimit + eps;
        return;
      }
      if(vToUppBnd<=eps)
      {
        aLow = -aLimit;
        aUpp = -aLimit + eps;
        return;
      }
      // Region I comparison.
      double RegionI = x + 0.5 * v * v/aLimit;
      if(RegionI>=xUpp)
      {
        aLow = -aLimit;
        aUpp = -aLimit + eps;
      }
    }
  }
  else
  {
    if(vToUppBnd<=eps)
    {
      aLow = -aLimit;
      aUpp = -aLimit + eps;
      return;
    }
    if(xToLeftBnd<=eps)
    {
      aLow = aLimit;
      aUpp = aLimit + eps;
      return;
    }
    double RegionII = x - 0.5 * v * v/aLimit;
    if(RegionII<=xLow)
    {
      aLow = aLimit;
      aUpp = aLimit + eps;
      return;
    }
    if(vToDownBnd<=eps)
    {
      aLow = aLimit;
      aUpp = aLimit + eps;
      return;
    }
  }

  aLow = -aLimit;
  aUpp = aLimit;
  return;
}


static std::vector<int> ActiveJacobianIndex(const std::vector<int> & _ContactActStatus)
{
  // This function is used to get out all the actively indepedent rows
  std::vector<int> ActJacIndices;
  int CurIndex = 0;
  for (int i = 0; i < _ContactActStatus.size(); i++)
  {
    switch (_ContactActStatus[i])
    {
      case 1:
      {
        ActJacIndices.push_back(CurIndex);
        CurIndex = CurIndex + 1;
      }
      break;
      case -1:
      {
        CurIndex = CurIndex + 1;
      }
      break;
      default:
      break;
    }
  }
  return ActJacIndices;
}

static int QPSolver(Robot & _SimRobot, const std::vector<double> & RobotVelocity, std::vector<double> & Tau, std::vector<double> & qddot)
{
  /*
      Variables to be optimized:
      0. qddot           DOF
      1. Tau             DOF - 6
      2. f               ConeAllUnits.size()
      3. eta             ContactActNo * 3
  */

  std::vector<int> ContactActStatus(ContactPointNo);
  int ContactPtInd = 0;
  int ContactActNo = 0;
  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    int LinkiPNo = RobotLinkInfo[i].LocalContacts.size();
    int LinkiPActNo = 0;
    for (int j = 0; j < LinkiPNo; j++)
    {
      switch (RobotContactInfo[i].LocalContactStatus[j])
      {
        case 0:
        {
          // This means that the current contact point should not be active.
          ContactActStatus[ContactPtInd] = 0;
        }
        break;
        case 1:
        {
          // This means that the current contact points should be active.
          if(LinkiPActNo<3)
          {
            LinkiPActNo = LinkiPActNo + 1;
            ContactActStatus[ContactPtInd] = 1;
            ContactActNo = ContactActNo + 1;
          }
          else
          {
            ContactActStatus[ContactPtInd] = -1;    // This means that the current contact is active but its Jacobian matrix is not considered.
          }
        }
        break;
        default:
        {
          // It should never enter this loop.
        }
        break;
      }
      ContactPtInd = ContactPtInd + 1;
    }
  }

  // Number of Variables to be optimized and Constraints
  int n = DOF + DOF - 6 + ConeAllUnits.size() + ContactActNo * 3;
  int neF = 0;                                  // Objective
  neF = neF + DOF;                              // Dynamics
  neF = neF + ContactActNo * 3;                 // Contact acceleration.

  std::vector<double> x_soln(n);
  double ConsVioVal = 1;
  int info;

  // Now let's focus on the linear cost term
  Vector ConfigRef(RobotConfigRef);
  Vector PosOffset = Kpos * (_SimRobot.q - ConfigRef);
  for (int i = 0; i < 6; i++)
  {
    PosOffset[i] = 0.0;
  }
  Vector RobotVelocityVec(RobotVelocity);
  Vector VelOffset = Kvel * RobotVelocityVec;
  Vector qddotOffset = PosOffset + VelOffset;

  try {
    GRBEnv env = GRBEnv();

    GRBModel model = GRBModel(env);

    // Create variables

    std::vector<GRBVar> OptVariables;
    OptVariables.reserve(n);

    int VarInd = 0;
    double accScale = 1.0;
    double torScale = 1.0;

    // 0.qddot
    for (int i = 0; i < 6; i++)
    {
      std::string x_name = "x" + std::to_string(VarInd);
      GRBVar x_i = model.addVar(-1.0 * InfVal, InfVal, 0.0, GRB_CONTINUOUS, x_name);
      OptVariables.push_back(x_i);
      VarInd += 1;
    }

    // for (int i = 6; i < DOF; i++)
    // {
    //   std::string x_name = "x" + std::to_string(VarInd);
    //   double accMax_i = accScale * _SimRobot.accMax[i];
    //   GRBVar x_i = model.addVar(-1.0 * accMax_i, accMax_i, 0.0, GRB_CONTINUOUS, x_name);
    //   OptVariables.push_back(x_i);
    //   VarInd += 1;
    // }

    for (int i = 6; i < DOF; i++)
    {
      std::string x_name = "x" + std::to_string(VarInd);
      double accMax_i = accScale * _SimRobot.accMax[i];
      double aLow, aUpp;
      AccBndGene(_SimRobot.q[i], _SimRobot.dq[i], _SimRobot.qMin(i), _SimRobot.qMax(i), _SimRobot.velMax[i], _SimRobot.accMax[i], aLow, aUpp);
      GRBVar x_i = model.addVar(aLow, aUpp, 0.0, GRB_CONTINUOUS, x_name);
      OptVariables.push_back(x_i);
      VarInd += 1;
    }

    // 1.tau
    for (int i = 0; i < DOF - 6; i++)
    {
      std::string x_name = "x" + std::to_string(VarInd);
      double torMax_i = torScale * _SimRobot.torqueMax[i + 6];
      GRBVar x_i = model.addVar(-1.0 * torMax_i, torMax_i, 0.0, GRB_CONTINUOUS, x_name);
      OptVariables.push_back(x_i);
      VarInd += 1;
    }

    // 2. f
    for (int i = 0; i < ConeAllUnits.size(); i++)
    {
      std::string x_name = "x" + std::to_string(VarInd);
      GRBVar x_i = model.addVar(0.0, InfVal, 0.0, GRB_CONTINUOUS, x_name);
      OptVariables.push_back(x_i);
      VarInd += 1;
    }

    // 3. eta
    for (int i = 0; i < 3 * ContactActNo; i++)
    {
      std::string x_name = "x" + std::to_string(VarInd);
      GRBVar x_i = model.addVar(-ConAccTol, ConAccTol, 0.0, GRB_CONTINUOUS, x_name);
      OptVariables.push_back(x_i);
      VarInd += 1;
    }

    /*
      Set objective
    */
    GRBQuadExpr obj = 0;
    int StartInd = 0;
    for (int i = StartInd; i < StartInd + DOF; i++)
    {
      // qddot
      obj+=(1.0 + Kqddot) * OptVariables[i] * OptVariables[i];
      obj+=2.0 * qddotOffset[i] * OptVariables[i];
    }
    StartInd = StartInd + DOF;
    for (int i = StartInd; i < StartInd + DOF - 6; i++)
    {
      // tau
      obj+=Ku * OptVariables[i] * OptVariables[i];
    }
    StartInd = StartInd + DOF - 6;
    for (int i = StartInd; i < StartInd + ConeAllUnits.size(); i++)
    {
      // f
      obj+=Kf * OptVariables[i] * OptVariables[i];
    }
    StartInd = StartInd + ConeAllUnits.size();
    for (int i =StartInd; i <StartInd + 3 * ContactActNo; i++)
    {
      // eta
      obj+=Keta * OptVariables[i] * OptVariables[i];
    }
    double Offset = qddotOffset.dot(qddotOffset);
    obj+=Offset;
    model.setObjective(obj);

    // Next step is to add constraint: there are two types of constraints
    // 0. Dynamics constraint

    // 1. Dynamics constraints
    NewtonEulerSolver NEDynamics(_SimRobot);
    Vector3 GraAcc(0.0, 0.0, -9.81);
    NEDynamics.SetGravityWrenches(GraAcc);
    Matrix D_q;
    NEDynamics.CalcKineticEnergyMatrix(D_q);
    Vector CG;
    NEDynamics.CalcResidualTorques(CG);

    Matrix pDynpf;
    pDynpf.resize(DOF, ConeAllUnits.size());
    pDynpf.setZero();
    int UnitIndex = 0;
    for (int i = 0; i < ConeAllUnits.size()/EdgeNo; i++)
    {
      Matrix J_i = ActJacobians[i];
      for (int j = 0; j < EdgeNo; j++)
      {
        Matrix ConeUnit;
        ConeUnit.resize(1, 3);
        ConeUnit(0, 0) = ConeAllUnits[UnitIndex].x;
        ConeUnit(0, 1) = ConeAllUnits[UnitIndex].y;
        ConeUnit(0, 2) = ConeAllUnits[UnitIndex].z;

        Matrix J_iConeUnit;
        J_iConeUnit.mul(ConeUnit, J_i);
        for (int k = 0; k < DOF; k++)
        {
          pDynpf(k, UnitIndex) = J_iConeUnit(0, k);
        }
        UnitIndex += 1;
      }
    }

    int ConsInd = 0;
    for (int i = 0; i < DOF; i++)
    {
      GRBLinExpr Dyn_left_i = 0;
      for (int j = 0; j < DOF; j++)
      {
        Dyn_left_i+=D_q(i,j) * OptVariables[j];
      }
      Dyn_left_i+=CG[i];

      GRBLinExpr Dyn_right_i = 0;
      for (int j = 0; j < ConeAllUnits.size(); j++)
      {
        Dyn_right_i+=pDynpf(i,j) * OptVariables[DOF + DOF - 6 + j];
      }
      if(i>=6)
      {
        Dyn_right_i+=OptVariables[DOF + i - 6];
      }
      std::string cons_name = "c" + std::to_string(ConsInd);
      model.addConstr(Dyn_left_i==Dyn_right_i, cons_name);
      ConsInd+=1;
    }

    // 1. Contact Acceleration constraint
    Config qdot_t(RobotVelocity);

    std::vector<double> Jdotqdot(3 * ContactActNo);
    int Jdotqdotindex = 0;
    int ContactActIndex = 0;
    for (int i = 0; i < RobotLinkInfo.size(); i++)
    {
      int LinkiPNo = RobotLinkInfo[i].LocalContacts.size();
      for (int j = 0; j < LinkiPNo; j++)
      {
        switch(ContactActStatus[ContactActIndex])
        {
          case 1:
          {
            Matrix* Hp[3];
            Matrix Hp_x(DOF,DOF);           Matrix Hp_y(DOF,DOF);           Matrix Hp_z(DOF,DOF);
            Hp[0] = &Hp_x;                  Hp[1] = &Hp_y;                  Hp[2] = &Hp_z;
            _SimRobot.GetPositionHessian(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, Hp);
            // Here Hp is a square matrix should be multiplied by qdot to get dJ/dt
            Vector dJxdt, dJydt, dJzdt;
            Hp[0]->mul(qdot_t, dJxdt);      // Should be a DOF * 1 row vector
            Hp[1]->mul(qdot_t, dJydt);
            Hp[2]->mul(qdot_t, dJzdt);

            double dJxdtqdot = dJxdt.dot(qdot_t);
            Jdotqdot[Jdotqdotindex] = dJxdtqdot;
            Jdotqdotindex = Jdotqdotindex + 1;

            double dJydtqdot = dJydt.dot(qdot_t);
            Jdotqdot[Jdotqdotindex] = dJydtqdot;
            Jdotqdotindex = Jdotqdotindex + 1;

            double dJzdtqdot = dJzdt.dot(qdot_t);
            Jdotqdot[Jdotqdotindex] = dJzdtqdot;
            Jdotqdotindex = Jdotqdotindex + 1;
          }
          break;
          default:
          break;
        }
        ContactActIndex = ContactActIndex + 1;
      }
    }

    Matrix J;
    J.resize(3 * ContactActNo, DOF);
    J.setZero();

    std::vector<int> ActJacobianIndex = ActiveJacobianIndex(ContactActStatus);
    for (int i = 0; i < ActJacobianIndex.size(); i++)
    {
      Matrix J_i = ActJacobians[ActJacobianIndex[i]];
      int J_i_ind = 3 * i;
      for (int j = 0; j < 3; j++)
      {
        for (int k = 0; k < J_i.n; k++)
        {
          J(J_i_ind + j, k) = J_i(j,k);
        }
      }
    }

    for (int i = 0; i < 3 * ContactActNo; i++)
    {
      GRBLinExpr ConAcc_left_i = 0;
      for (int j = 0; j < DOF; j++)
      {
        ConAcc_left_i+=J(i,j)*OptVariables[j];
      }
      ConAcc_left_i+=Jdotqdot[i];

      GRBLinExpr ConAcc_right_i = 0;
      ConAcc_right_i+=OptVariables[DOF + DOF - 6 + ConeAllUnits.size() + i];
      std::string cons_name = "c" + std::to_string(ConsInd);
      model.addConstr(ConAcc_left_i==ConAcc_right_i, cons_name);
      ConsInd+=1;
    }

    model.optimize();

    cout << "Objective Value: " << model.get(GRB_DoubleAttr_ObjVal)<< endl;

    for (int i = 0; i < n; i++)
    {
      x_soln[i] = OptVariables[i].get(GRB_DoubleAttr_X);
    }
    // This step is used to check the feasibility of the answer.
    std::vector<double> forceMag(ConeAllUnits.size());
    std::vector<double> eta(3 * ContactActNo);
    for (int i = 0; i < DOF; i++)
    {
      qddot[i] = x_soln[i];
    }
    for (int i = 0; i < DOF - 6; i++)
    {
      Tau[i + 6] = x_soln[DOF + i];
    }
    for (int i = 0; i < ConeAllUnits.size(); i++)
    {
      forceMag[i] = x_soln[DOF + DOF - 6 + i];
    }
    for (int i = 0; i < 3 * ContactActNo; i++)
    {
      eta[i] = x_soln[DOF + DOF - 6 + ConeAllUnits.size() + i];
    }

    // This following part is used for debugging.
    Vector qddot_(qddot);
    Vector Tau_(Tau);
    Vector forceMag_(forceMag);
    Vector eta_(eta);
    Vector JqddotRes;
    J.mul(qddot_, JqddotRes);

    std::vector<double> ConsVio(DOF + J.m);

    Vector D_q_qddot;
    D_q.mul(qddot_, D_q_qddot);
    Vector JTLambda;
    pDynpf.mul(forceMag_, JTLambda);
    for (int i = 0; i < DOF; i++)
    {
      double Dyn_i;
      Dyn_i = D_q_qddot[i] + CG[i] - JTLambda[i] - Tau[i];
      ConsVio[i] = Dyn_i * Dyn_i;
    }

    for (int i = DOF; i < DOF + J.m; i++)
    {
      double Jacc_i = JqddotRes[i - DOF] + Jdotqdot[i - DOF] - eta_[i - DOF];
      ConsVio[i] = Jacc_i * Jacc_i;
    }

    ConsVioVal = *std::max_element(ConsVio.begin(), ConsVio.end());

  } catch(GRBException e)
  {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...)
  {
    cout << "Exception during optimization" << endl;
  }

  if(ConsVioVal<epsTol)
  {
    info = 1;
  }
  else
  {
    info = 0;
  }

  return info;
}

std::vector<double> StabilizingControllerGRB(const Robot& SimRobot, const std::vector<Matrix> & _ActJacobians, const std::vector<Vector3>& _ConeAllUnits, const int & _EdgeNo, const int& _DOF, const double& dt, std::vector<Config>& qTraj, std::vector<Config> & qdotTraj, std::vector<Config> & qddotTraj, std::vector<Config> & qTrajAct, std::vector<Config> & qdotTrajAct, int & _QPStatus, std::vector<LinkInfo> & _RobotLinkInfo, std::vector<ContactStatusInfo> & _RobotContactInfo, std::vector<double> & _RobotConfigRef, const int & _ContactPointNo, const int & StepIndex)
{
  Robot _SimRobot = SimRobot;
  ConeAllUnits = _ConeAllUnits;
  ActJacobians = _ActJacobians;
  EdgeNo = _EdgeNo;
  DOF = _DOF;

  RobotLinkInfo = _RobotLinkInfo;
  RobotContactInfo = _RobotContactInfo;
  RobotConfigRef = _RobotConfigRef;
  ContactPointNo = _ContactPointNo;

  double VelLimit = 3.0;

  qTrajAct.push_back(_SimRobot.q);
  std::vector<double> RobotVelocityCur(DOF);

  switch (StepIndex)
  {
    case 0:
    {
      for (int i = 0; i < _SimRobot.q.size(); i++)
      {
        RobotVelocityCur[i] = _SimRobot.dq[i];
        if(RobotVelocityCur[i]<_SimRobot.velMin(i))
        {
          RobotVelocityCur[i] = _SimRobot.velMin(i);
        }
        if(RobotVelocityCur[i]>_SimRobot.velMax(i))
        {
          RobotVelocityCur[i] = _SimRobot.velMax(i);
        }
      }
    }
    break;
    default:
    {
      for (int i = 0; i < 6; i++)
      {
        double RobotVelocity_i = (qTrajAct[qTrajAct.size()-1][i] - qTrajAct[qTrajAct.size()-2][i])/dt;
        if((RobotVelocity_i<-VelLimit)||(RobotVelocity_i>VelLimit))
        {
          RobotVelocityCur[i] = 0.0;
        }
        else
        {
          RobotVelocityCur[i] = RobotVelocity_i;
        }
      }
      for (int i = 6; i < _SimRobot.q.size(); i++)
      {
        double RobotVelocity_i = (qTrajAct[qTrajAct.size()-1][i] - qTrajAct[qTrajAct.size()-2][i])/dt;
        switch (i)
        {
          case 10:
          {
              RobotVelocityCur[i] = _SimRobot.dq[i];
          }
          break;
          case 11:
          {
              RobotVelocityCur[i] = _SimRobot.dq[i];
          }
          break;
          case 16:
          {
              RobotVelocityCur[i] = _SimRobot.dq[i];
          }
          break;
          case 17:
          {
            RobotVelocityCur[i] = _SimRobot.dq[i];
          }
          break;
          default:
          {
            RobotVelocityCur[i] = RobotVelocity_i;
          }
          break;
        }
        if(RobotVelocityCur[i]<_SimRobot.velMin(i))
        {
          RobotVelocityCur[i] = _SimRobot.velMin(i);
        }
        if(RobotVelocityCur[i]>_SimRobot.velMax(i))
        {
          RobotVelocityCur[i] = _SimRobot.velMax(i);
        }
      }
    }
    break;
  }
  _SimRobot.dq = RobotVelocityCur;

  Config RobotVelocityCurrent(RobotVelocityCur);
  qdotTrajAct.push_back(RobotVelocityCurrent);

  std::vector<double> qNew;
  std::vector<double> Tau(DOF), qddot(DOF);
  int QPStatus = QPSolver(_SimRobot, RobotVelocityCur, Tau, qddot);
  std:printf("QPStatus: %d \n", QPStatus);
  _QPStatus = QPStatus;

  std::vector<double> qDes(DOF), qdotDes(DOF);
  std::vector<double> qRef = qTraj[qTraj.size()-1];
  std::vector<double> qdotRef = qdotTraj[qdotTraj.size()-1];

  std::vector<double> qddotDes(DOF);

  // 1. Damping
  double DampCostVal = 0.0;
  for (int i = 0; i < DOF; i++)
  {
    DampCostVal+= (qddot[i] + Kvel * RobotVelocityCurrent[i]) * (qddot[i] + Kvel * RobotVelocityCurrent[i]);
  }

  // 2. Regularization
  double RegCostVal = 0.0;
  for (int i = 0; i < DOF; i++)
  {
    RegCostVal+=qddot[i] * qddot[i];
  }

  switch (QPStatus)
  {
    case 1:
    {
      // This means that the current QP Controller indeed returns an optimal solution.
      for (int i = 0; i < DOF; i++)
      {
        qddotDes[i] = qddot[i];
      }
    }
    break;
    default:
    {
      for (int i = 0; i < DOF; i++)
      {
        qddotDes[i] = 0.0;
      }
    }
    break;
  }

  for (int i = 0; i < DOF; i++)
  {
    double qDes_i = qRef[i] + qdotRef[i] * dt + 0.5 * qddotDes[i] * dt * dt;
    double qdotDes_i = qdotRef[i] + qddotDes[i] * dt;

    if(qDes_i<=_SimRobot.qMin(i))
    {
      std::printf("\nConfig %d is below minimum value", i);
      qDes_i = _SimRobot.qMin(i);
      qdotDes_i = 0.0;
    }
    if(qDes_i>=_SimRobot.qMax(i))
    {
      std::printf("\nConfig %d is above maximum value", i);
      qDes_i = _SimRobot.qMax(i);
      qdotDes_i = 0.0;
    }
    if(qdotDes_i<=_SimRobot.velMin(i))
    {
      std::printf("\nVelocity %d is below minimum value", i);
      qdotDes_i = _SimRobot.velMin(i);
    }
    if(qdotDes_i>=_SimRobot.velMax(i))
    {
      std::printf("\nVelocity %d is above maximum value", i);
      qdotDes_i = _SimRobot.velMax(i);
    }
    qDes[i] = qDes_i;
    qdotDes[i] = qdotDes_i;
  }

  Config qDes_(qDes);
  Config qdotDes_(qdotDes);
  Config qddotDes_(qddotDes);
  qTraj.push_back(qDes_);
  qdotTraj.push_back(qdotDes_);
  qddotTraj.push_back(qddotDes_);

  qNew = qDes;

  return qNew;
}
