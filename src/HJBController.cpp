#include "NonlinearOptimizerInfo.h"
#include "CommonHeader.h"
#include <random>
#include "gurobi_c++.h"

// This function is used to compute the HJB tracking controller to stabilize the robot.
static Vector3 COMPos;
static Vector3 COMVel;
static std::vector<Matrix> ActJacobians;
static std::vector<Vector3> ConeUnits;
static int EdgeNumber;
static int DOF;

static std::vector<LinkInfo> RobotLinkInfo;
static std::vector<ContactStatusInfo> RobotContactInfo;
static int NumberOfContactPoints;

static Vector3 COMDes;
static Matrix Jcom;
static Vector3 aCOMOffset;

/*
    Total Objective Function:
                    Kcen * ||Jcom*qddot + aCOMOffset||_2^2 + Kqdot * ||qddot + Kdamp * qdot||_2^2 + Kqddot * ||qddot||_2^2
                    where aCOMOffset = Jcomdot * qdot + Kpos * (rCOM - rCOMDes) + Kvel * COMVel
*/

// QP Gains
static double Kcen = 2000000.0;
static double Kpos_x = 30.0;
static double Kpos_y = 30.0;
static double Kpos_z = 30.0;
static double Kvel_x = 5.0;
static double Kvel_y = 5.0;
static double Kvel_z = 5.0;

// A potential good option
// static double Kpos = 36.0;
// static double Kvel = 7.75;

static double Kqdot = 1.0;
static double Kdamp = 20.0;
// static double Kqddot = 0.0;
// static double Kdamp = 20.0;
static double Kqddot = 0.0;

static double ConAccTol = 0.01;

using namespace std;

static double epsTol = 1e-8;

static std::vector<int> ActiveJacobianIndex(const std::vector<int> & _ContactActStatus)
{
  // This function is used to get out all actively independent rows
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

static int HJBSolver(Robot & SimRobot, const Config & RobotVelocityCurrent, std::vector<double> & qddot)
{
  /*
      Variables to be optimized:
        0. qddot           DOF
        1. Tau             DOF - 6
        2. f               ConeUnits.size()
        3. eta             ContactActNo * 3
  */

  double InfVal = 10000.0;

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
  int n = DOF + DOF - 6 + ConeUnits.size() + ContactActNo * 3;
  int neF = 0;                                  // Objective
  neF = neF + DOF;                              // Dynamics
  neF = neF + ContactActNo * 3;                 // Contact acceleration.

  std::vector<double> x_soln(n);
  double ConsVioVal = 1;
  int info;

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
    for (int i = 0; i < ConeUnits.size(); i++)
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

    Matrix JcomTJcom;
    JcomTJcom.mulTransposeA(Jcom, Jcom);    // Now this matrix should be of size DOF x DOF

    /*
      Set objective: Centroidal Tracking + Damping + Regularization
    */
    GRBQuadExpr obj = 0;

    Matrix aCOMOffsetMat;
    aCOMOffsetMat.resize(1,3);
    aCOMOffsetMat(0,0) = aCOMOffset.x;
    aCOMOffsetMat(0,1) = aCOMOffset.y;
    aCOMOffsetMat(0,2) = aCOMOffset.z;

    Matrix aCOMOffsetTJcom;
    aCOMOffsetTJcom.mul(aCOMOffsetMat, Jcom);

    // printf("Jcom: \n");
    // MatrixPrintResult(Jcom);
    //
    // printf("aCOMOffsetMat: \n");
    // MatrixPrintResult(aCOMOffsetMat);
    //
    // printf("aCOMOffsetTJcom: \n");
    // MatrixPrintResult(aCOMOffsetTJcom);

    // 1. Centroidal Tracking
    for (int i = 0; i < DOF; i++)
    {
      for (int j = 0; j < DOF; j++)
      {
        obj+=Kcen * JcomTJcom(i,j) * OptVariables[i] * OptVariables[j];
      }
      obj+=Kcen * 2.0 * aCOMOffsetTJcom(0,i) * OptVariables[i];
    }
    obj+=Kcen * (aCOMOffset.x * aCOMOffset.x + aCOMOffset.y * aCOMOffset.y + aCOMOffset.z * aCOMOffset.z);

    // 2. Damping
    for (int i = 0; i < DOF; i++)
    {
      obj+=Kqdot * (OptVariables[i] + Kdamp * RobotVelocityCurrent[i]) * (OptVariables[i] + Kdamp * RobotVelocityCurrent[i]);
    }

    // 3. Acceleration Regularlization
    for (int i = 0; i < DOF; i++)
    {
      obj+=Kqddot * OptVariables[i] * OptVariables[i];
    }

    model.setObjective(obj);

    // Next step is to add constraint: there are two types of constraints

    // 0. Dynamics constraint
    NewtonEulerSolver NEDynamics(SimRobot);
    Vector3 GraAcc(0.0, 0.0, -9.81);
    NEDynamics.SetGravityWrenches(GraAcc);
    Matrix D_q;
    NEDynamics.CalcKineticEnergyMatrix(D_q);
    Vector CG;
    NEDynamics.CalcResidualTorques(CG);

    Matrix pDynpf;
    pDynpf.resize(DOF, ConeUnits.size());
    pDynpf.setZero();
    int UnitIndex = 0;
    for (int i = 0; i < ConeUnits.size()/EdgeNumber; i++)
    {
      Matrix J_i = ActJacobians[i];
      for (int j = 0; j < EdgeNumber; j++)
      {
        Matrix ConeUnit;
        ConeUnit.resize(1, 3);
        ConeUnit(0, 0) = ConeUnits[UnitIndex].x;
        ConeUnit(0, 1) = ConeUnits[UnitIndex].y;
        ConeUnit(0, 2) = ConeUnits[UnitIndex].z;

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
      for (int j = 0; j < ConeUnits.size(); j++)
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
    Config qdot_t = RobotVelocityCurrent;

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
      ConAcc_right_i+=OptVariables[DOF + DOF - 6 + ConeUnits.size() + i];
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
    std::vector<double> forceMag(ConeUnits.size());
    std::vector<double> eta(3 * ContactActNo);
    std::vector<double> Tau(DOF);
    for (int i = 0; i < DOF; i++)
    {
      qddot[i] = x_soln[i];
    }
    for (int i = 0; i < DOF - 6; i++)
    {
      Tau[i + 6] = x_soln[DOF + i];
    }
    for (int i = 0; i < ConeUnits.size(); i++)
    {
      forceMag[i] = x_soln[DOF + DOF - 6 + i];
    }
    for (int i = 0; i < 3 * ContactActNo; i++)
    {
      eta[i] = x_soln[DOF + DOF - 6 + ConeUnits.size() + i];
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
    return 0;
  } catch(...)
  {
    cout << "Exception during optimization" << endl;
    return 0;
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

std::vector<double> HJBController(const Robot & _SimRobot, const Vector3 & _COMPos, const Vector3 & _COMVel, const std::vector<Vector3> & COMDesVector, const std::vector<int> & StatusVector, const std::vector<Matrix> & _ActJacobians, const std::vector<Vector3>& _ConeUnits, const int & _EdgeNumber, const int & _DOF, const double & dt, std::vector<Config>& qTraj, std::vector<Config> & qdotTraj, std::vector<Config> & qddotTraj, std::vector<Config> & qTrajAct, std::vector<Config> & qdotTrajAct, Vector3 & COMPosdes, int & QPStatus, std::vector<LinkInfo> & _RobotLinkInfo, std::vector<ContactStatusInfo> & _RobotContactInfo, const int & _NumberOfContactPoints, const int & StepIndex, bool & SwitchFlag)
{
  Robot SimRobot = _SimRobot;
  COMPos = _COMPos;
  COMVel = _COMVel;
  ActJacobians = _ActJacobians;
  ConeUnits = _ConeUnits;
  EdgeNumber = _EdgeNumber;
  DOF = _DOF;

  RobotLinkInfo = _RobotLinkInfo;
  RobotContactInfo = _RobotContactInfo;
  NumberOfContactPoints = _NumberOfContactPoints;

  double VelLimit = 3.0;

  // First, takes into robot's current state.
  qTrajAct.push_back(SimRobot.q);
  std::vector<double> RobotVelocityCur(DOF);

  switch (StepIndex)
  {
    case 0:
    {
      for (int i = 0; i < SimRobot.q.size(); i++)
      {
        RobotVelocityCur[i] = SimRobot.dq[i];
        if(RobotVelocityCur[i]<SimRobot.velMin(i))
        {
          RobotVelocityCur[i] = SimRobot.velMin(i);
        }
        if(RobotVelocityCur[i]>SimRobot.velMax(i))
        {
          RobotVelocityCur[i] = SimRobot.velMax(i);
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
          // case 10:
          // {
          //     RobotVelocityCur[i] = _SimRobot.dq[i];
          // }
          // break;
          // case 11:
          // {
          //     RobotVelocityCur[i] = _SimRobot.dq[i];
          // }
          // break;
          // case 16:
          // {
          //     RobotVelocityCur[i] = _SimRobot.dq[i];
          // }
          // break;
          // case 17:
          // {
          //   RobotVelocityCur[i] = _SimRobot.dq[i];
          // }
          // break;
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
  Config RobotVelocityCurrent(RobotVelocityCur);
  qdotTrajAct.push_back(RobotVelocityCurrent);

  SimRobot.dq = RobotVelocityCurrent;

  // Here COMDes is desired center of mass position.
  int PIPStatus = 0;
  double COMDesx = 0.0;
  double COMDesy = 0.0;
  double COMDesz = 0.0;
  for (int i = 0; i < StatusVector.size(); i++)
  {
    COMDesx+= StatusVector[i] * 1.0 * COMDesVector[i].x;
    COMDesy+= StatusVector[i] * 1.0 * COMDesVector[i].y;
    COMDesz+= StatusVector[i] * 1.0 * COMDesVector[i].z;
    PIPStatus+= StatusVector[i];
  }
  switch (PIPStatus)
  {
    case 0:
    {
      // There is no need to track a COM position.
      COMDes = COMPos;
    }
    break;
    default:
    {
      // Tracking has to be conducted.
      COMDes.x = COMDesx/(1.0 * PIPStatus);
      COMDes.y = COMDesy/(1.0 * PIPStatus);
      COMDes.z = COMDesz/(1.0 * PIPStatus);
    }
    break;
  }

  COMDes.x = 0.13563739414658971;
  // COMDes.y = 0.13093482618578417;
  // COMDes.y = 0.14093482618578417;

  // COMDes.x = COMPos.x;
  COMDes.y = COMPos.y;
  // COMDes.z = COMPos.z;
  COMDes.z = 0.78386621712800796;

  // double TrackPosTol = 0.0025;    // 2.5mm
  //
  // if((COMDes.x - COMPos.x) * (COMDes.x - COMPos.x)<TrackPosTol * TrackPosTol)
  // {
  //   COMDes.x = COMPos.x;
  // }
  // if((COMDes.y - COMPos.y) * (COMDes.y - COMPos.y)<TrackPosTol * TrackPosTol)
  // {
  //   COMDes.y = COMPos.y;
  // }
  // if((COMDes.z - COMPos.z) * (COMDes.z - COMPos.z)<TrackPosTol * TrackPosTol)
  // {
  //   COMDes.z = COMPos.z;
  // }

  COMPosdes = COMDes;

  // Here the Centroidal Acceleration needs to be expressed explicitly.
  // aCOM = Jcom * qddot + dJcom/dt * qdot;
  SimRobot.GetCOMJacobian(Jcom);

  Matrix Hcom_x, Hcom_y, Hcom_z;
  SimRobot.GetCOMHessian(Hcom_x, Hcom_y, Hcom_z);

  Vector Hcom_xqdot, Hcom_yqdot, Hcom_zqdot;
  Hcom_x.mul(RobotVelocityCurrent, Hcom_xqdot);
  Hcom_y.mul(RobotVelocityCurrent, Hcom_yqdot);
  Hcom_z.mul(RobotVelocityCurrent, Hcom_zqdot);

  double Jcomdotqdotx = Hcom_xqdot.dot(RobotVelocityCurrent);
  double Jcomdotqdoty = Hcom_yqdot.dot(RobotVelocityCurrent);
  double Jcomdotqdotz = Hcom_zqdot.dot(RobotVelocityCurrent);

  Vector3 Jcomdotqdot(Jcomdotqdotx, Jcomdotqdoty, Jcomdotqdotz);

  aCOMOffset.x = Kpos_x * (COMPos.x - COMDes.x) + Kvel_x * COMVel.x;
  aCOMOffset.y = Kpos_y * (COMPos.y - COMDes.y) + Kvel_y * COMVel.y;
  aCOMOffset.z = Kpos_z * (COMPos.z - COMDes.z) + Kvel_z * COMVel.z;

  aCOMOffset = aCOMOffset + Jcomdotqdot;

  std::vector<double> qNew;

  std::vector<double> qddot(DOF);
  QPStatus = HJBSolver(SimRobot, RobotVelocityCurrent, qddot);
  std:printf("QPStatus: %d \n", QPStatus);

  // 0. Centroidal Tracking Cost
  Vector3 CentTrackCost = aCOMOffset;
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < DOF; j++)
    {
      CentTrackCost[i]+=Jcom(i,j) * qddot[j];
    }
  }

  double CentTrackCostVal = CentTrackCost[0] * CentTrackCost[0] + CentTrackCost[1] * CentTrackCost[1] + CentTrackCost[2] * CentTrackCost[2];
  printf("Position Tracking Value: %f\n", Kcen * CentTrackCostVal);
  Vector3 PosTracking(0.0, 0.0, 0.0);
  PosTracking.x = (COMPos.x - COMDes.x);
  PosTracking.y = (COMPos.y - COMDes.y);
  PosTracking.z = (COMPos.z - COMDes.z);

  double PosTrackCostVal = PosTracking[0] * PosTracking[0] + PosTracking[1] * PosTracking[1] + PosTracking[2] * PosTracking[2];
  printf("Position Difference Value: %f cm\n", sqrt(PosTrackCostVal) * 100.0);

  // 1. Damping
  double DampCostVal = 0.0;
  for (int i = 0; i < DOF; i++)
  {
    DampCostVal+=Kqdot * (qddot[i] + Kdamp * RobotVelocityCurrent[i]) * (qddot[i] + Kdamp * RobotVelocityCurrent[i]);
  }
  printf("Damping Value: %f\n", DampCostVal);

  // 2. Regularization
  double RegCostVal = 0.0;
  for (int i = 0; i < DOF; i++)
  {
    RegCostVal+=qddot[i] * qddot[i];
  }
  printf("Regularization Value: %f\n", Kqddot * RegCostVal);

  std::vector<double> qDes(DOF), qdotDes(DOF), qddotDes(DOF);
  std::vector<double> qRef = qTraj[qTraj.size()-1];
  std::vector<double> qdotRef = qdotTraj[qdotTraj.size()-1];

  switch (QPStatus)
  {
    case 1:
    {
      // This means that the current QP Controller indeed returns an optimal solution.
      qddotDes = qddot;
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

  // Now it is time to update the robot's commanded configuration.
  for (int i = 0; i < DOF; i++)
  {
    double qDes_i, qdotDes_i;
    qDes_i = qRef[i] + qdotRef[i] * dt + 0.5 * qddot[i] * dt * dt;
    qdotDes_i = qdotRef[i] + qddot[i] * dt;
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
  Config qddotDes_(qddot);
  qTraj.push_back(qDes_);
  qdotTraj.push_back(qdotDes_);
  qddotTraj.push_back(qddotDes_);

  qNew = qDes;
  return qNew;
}
