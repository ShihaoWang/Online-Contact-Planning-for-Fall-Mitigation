#include "NonlinearOptimizerInfo.h"
#include "CommonHeader.h"
#include <random>
#include "gurobi_c++.h"

// This function is used to compute the stabilizing controller to regain balance for the disturbed robot.

static Robot SimRobot;
static std::vector<LinkInfo> RobotLinkInfo;
static std::vector<ContactStatusInfo> RobotContactInfo;
static int ConeEdgeNumber;
static int NumberOfContactPoints;
static std::vector<double> RobotConfigRef;
static int DOF;

static std::vector<Vector3> ConeAllUnits;
static std::vector<Matrix> ActJacobians;

// QP Gains
static double Kpos = 0.0;
static double Kvel = 10.0;
static double Kqddot = 5.0;
static double Ku = 0.0;
static double Kf = 0.0;
static double Keta = 0.0;

static double ConAccTol = 0.1;
static double epsTol = 1e-8;
using namespace std;

static double InfVal = 10000.0;

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

static bool QPSolver(std::vector<double> & qddot)
{
  /*
      Variables to be optimized:
      0. qddot           DOF
      1. Tau             DOF - 6
      2. f               ConeAllUnits.size()
      3. eta             ContactActNo * 3
  */

  std::vector<int> ContactActStatus(NumberOfContactPoints);
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
          if(LinkiPActNo<4)
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
  bool info = true;

  // Now let's focus on the linear cost term
  Vector ConfigRef(RobotConfigRef);
  Vector PosOffset = Kpos * (SimRobot.q - ConfigRef);
  for (int i = 0; i < 6; i++)
  {
    PosOffset[i] = 0.0;
  }
  Vector VelOffset = Kvel * SimRobot.dq;
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
    for (int i = 6; i < DOF; i++)
    {
      std::string x_name = "x" + std::to_string(VarInd);
      double accMax_i = accScale * SimRobot.accMax[i];
      GRBVar x_i = model.addVar(-1.0 * accMax_i, accMax_i, 0.0, GRB_CONTINUOUS, x_name);
      OptVariables.push_back(x_i);
      VarInd += 1;
    }

    // 1.tau
    for (int i = 0; i < DOF - 6; i++)
    {
      std::string x_name = "x" + std::to_string(VarInd);
      double torMax_i = torScale * SimRobot.torqueMax[i + 6];
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

    model.setObjective(obj);

    // Next step is to add constraint: there are two types of constraints
    // 0. Dynamics constraint

    // 1. Dynamics constraints
    NewtonEulerSolver NEDynamics(SimRobot);
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
    for (int i = 0; i < ConeAllUnits.size()/ConeEdgeNumber; i++)
    {
      Matrix J_i = ActJacobians[i];
      for (int j = 0; j < ConeEdgeNumber; j++)
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
    Config qdot_t = SimRobot.dq;

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
            SimRobot.GetPositionHessian(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, Hp);
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
    try
    {
      double Offset = qddotOffset.dot(qddotOffset);
      cout << "Objective Value: " << model.get(GRB_DoubleAttr_ObjVal) + Offset << endl;
    }
    catch (...)
    {
      std::printf("QP Stabilizing Controller Fails!\n");
      return false;
    }
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
    std::vector<double> Tau(DOF);
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
    info = true;
  }
  else
  {
    info = false;
  }

  return info;
}

std::vector<double> QPController(std::vector<Config> & qTraj, std::vector<Config> & qdotTraj, std::vector<Config> & qddotTraj, std::vector<Config> & qTrajAct, std::vector<Config> & qdotTrajAct, int & QPStatus, const std::vector<Matrix> & ActJacobian, const std::vector<Vector3>& ConeAllUnit, ParaStructure & ParaStruct)
{
  // This function is used to calculate the stabilizing controller with a QP formulation.
  SimRobot = ParaStruct.SimRobot;
  RobotLinkInfo = ParaStruct.RobotLinkInfo;
  RobotContactInfo = ParaStruct.RobotContactInfo;
  ConeEdgeNumber = ParaStruct.ConeEdgeNumber;
  double dt = ParaStruct.dt;
  NumberOfContactPoints = ParaStruct.NumberOfContactPoints;
  DOF = ParaStruct.DOF;
  RobotConfigRef = ParaStruct.RobotConfigRef;

  ConeAllUnits = ConeAllUnit;
  ActJacobians = ActJacobian;

  std::vector<double> qNew;
  std::vector<double> qddot(DOF);
  QPStatus = QPSolver(qddot);
  std::printf("%s", QPStatus ? "QPStatus: true\n" : "QPStatus: false\n");

  std::vector<double> qDes(DOF), qdotDes(DOF);
  std::vector<double> qRef = qTraj[qTraj.size()-1];
  std::vector<double> qdotRef = qdotTraj[qdotTraj.size()-1];

  std::vector<double> qddotDes(DOF);

  switch (QPStatus)
  {
    case false:
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

    if(qDes_i<=SimRobot.qMin(i))
    {
      std::printf("\nConfig %d is below minimum value", i);
      qDes_i = SimRobot.qMin(i);
      qdotDes_i = 0.0;
    }
    if(qDes_i>=SimRobot.qMax(i))
    {
      std::printf("\nConfig %d is above maximum value", i);
      qDes_i = SimRobot.qMax(i);
      qdotDes_i = 0.0;
    }
    if(qdotDes_i<=SimRobot.velMin(i))
    {
      std::printf("\nVelocity %d is below minimum value", i);
      qdotDes_i = SimRobot.velMin(i);
    }
    if(qdotDes_i>=SimRobot.velMax(i))
    {
      std::printf("\nVelocity %d is above maximum value", i);
      qdotDes_i = SimRobot.velMax(i);
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

  qTrajAct.push_back(SimRobot.q);
  qdotTrajAct.push_back(SimRobot.dq);

  qNew = qDes;
  return qNew;
}
