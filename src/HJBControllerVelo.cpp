#include "NonlinearOptimizerInfo.h"
#include "CommonHeader.h"
#include <random>
#include <numeric>

#include "gurobi_c++.h"

static Vector3 COMPos;
static std::vector<Matrix> ActJacobians;
static int DOF;

static std::vector<LinkInfo> RobotLinkInfo;
static std::vector<ContactStatusInfo> RobotContactInfo;
static int NumberOfContactPoints;

static Vector3 COMDes;
static Vector3 COMVelDes;
static Matrix Jcom;
static std::vector<double> RobotVelocityRef;

static Matrix B_q;
static double Kcen = 10.0;
static double Kdamp = 1.0;
static double Kreg = 1.0;

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

static int HJBVelocitySolver(const Robot & SimRobot, const double & dt, std::vector<double> & qdotNew)
{
  /*
    Variables to be optimized:    qdot           DOF
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
  int n = DOF;
  std::vector<double> x_soln(n);
  double ConsVioVal = 1;
  int info;
  double epsTol = 1e-8;
  double InfVal = 10000.0;

  try
  {
    GRBEnv env = GRBEnv();

    GRBModel model = GRBModel(env);

    // Create variables

    std::vector<GRBVar> OptVariables;
    OptVariables.reserve(n);

    int VarInd = 0;

    // 0.qdot
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
      double velMax_i = SimRobot.velMax[i];
      GRBVar x_i = model.addVar(-1.0 * velMax_i, velMax_i, 0.0, GRB_CONTINUOUS, x_name);
      OptVariables.push_back(x_i);
      VarInd += 1;
    }

    /*
        Set objective: Centroidal Tracking + Damping + Regularization
    */

    GRBQuadExpr obj = 0;

    // 0. Tracking
    for (int i = 0; i < 3; i++)
    {
      GRBLinExpr CenVel_i = 0;
      for (int j = 0; j < DOF; j++)
      {
        CenVel_i += Jcom(i,j) * OptVariables[j];
      }
      obj+= (CenVel_i + Kcen * (COMPos[i] - COMDes[i])) * (CenVel_i + Kcen * (COMPos[i] - COMDes[i]));
    }

    // 1. Damping
    for (int i = 0; i < 3; i++)
    {
      GRBLinExpr CenVel_i = 0;
      for (int j = 0; j < DOF; j++)
      {
        CenVel_i += Jcom(i,j) * OptVariables[j];
      }
      obj+= Kdamp * CenVel_i * CenVel_i;
    }

    for (int i = 0; i < DOF; i++)
    {
      obj+= Kreg * OptVariables[i] * OptVariables[i];
    }

    model.setObjective(obj);

    // Constraint1: Contact Velocity constraint
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

    int ConsInd = 0;
    for (int i = 0; i < 3 * ContactActNo; i++)
    {
      GRBLinExpr ConAcc_left_i = 0;
      for (int j = 0; j < DOF; j++)
      {
        ConAcc_left_i+=J(i,j)*OptVariables[j];
      }
      GRBLinExpr ConAcc_right_i = 0;
      std::string cons_name = "c" + std::to_string(ConsInd);
      model.addConstr(ConAcc_left_i==ConAcc_right_i, cons_name);
      ConsInd+=1;
    }

    // // Constraint2: Centroidal Velocity Constraint
    // for (int i = 0; i < 3; i++)
    // {
    //   GRBLinExpr CenVel_left_i = 0;
    //   for (int j = 0; j < DOF; j++)
    //   {
    //     CenVel_left_i+=Jcom(i,j) * OptVariables[j];
    //   }
    //   GRBLinExpr CenVel_right_i = COMVelDes[i];
    //   std::string cons_name = "c" + std::to_string(ConsInd);
    //   model.addConstr(CenVel_left_i==CenVel_right_i, cons_name);
    //   ConsInd+=1;
    // }

    model.optimize();

    double ObjVal = model.get(GRB_DoubleAttr_ObjVal);

    cout << "Objective Value: " << ObjVal<< endl;


    for (int i = 0; i < n; i++)
    {
      x_soln[i] = OptVariables[i].get(GRB_DoubleAttr_X);
    }

    // This step is used to check the feasibility of the answer.
    for (int i = 0; i < DOF; i++)
    {
      qdotNew[i] = x_soln[i];
    }

    // This following part is used for debugging.
    Vector qdot_(qdotNew);
    Vector JqdotRes;
    J.mul(qdot_, JqdotRes);

    Vector JcomqdotRes;
    Jcom.mul(qdot_, JcomqdotRes);
    std::vector<double> ConsVio(3 * ContactActNo);
    for (int i = 0; i < 3 * ContactActNo; i++)
    {
      double Jacdot_i = JqdotRes[i];
      ConsVio[i] = Jacdot_i * Jacdot_i;
    }

    ConsVioVal = *std::max_element(ConsVio.begin(), ConsVio.end());
  }
  catch(GRBException e)
  {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
    return 0;
  }
  catch(...)
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

std::vector<double> HJBControllerVelo(const Robot & _SimRobot, const Vector3 & _COMPos, const std::vector<Vector3> & COMDesVector, const std::vector<int> & StatusVector, const int & _DOF, const double & dt, const std::vector<Matrix> & _ActJacobians, std::vector<Config>& qTraj, std::vector<Config> & qdotTraj, std::vector<Config> & qddotTraj, std::vector<Config> & qTrajAct, std::vector<Config> & qdotTrajAct, Vector3 & _COMDes, int & _QPStatus, std::vector<LinkInfo> & _RobotLinkInfo, std::vector<ContactStatusInfo> & _RobotContactInfo, const int & _NumberOfContactPoints, const int & StepIndex)
{
  Robot SimRobot = _SimRobot;
  COMPos = _COMPos;
  ActJacobians = _ActJacobians;
  DOF = _DOF;

  RobotLinkInfo = _RobotLinkInfo;
  RobotContactInfo = _RobotContactInfo;
  NumberOfContactPoints = _NumberOfContactPoints;

  double RollAngVelLimit = 3.0;
  qTrajAct.push_back(SimRobot.q);
  std::vector<double> RobotVelocityCur(DOF);
  for (int i = 0; i < DOF; i++)
  {
    RobotVelocityCur[i] = SimRobot.dq[i];
  }

  // for (int i = 0; i < DOF; i++)
  // {
  //   RobotVelocityCur[i] = SimRobot.dq[i];
  // }
  //
  // switch (StepIndex)
  // {
  //   case 0:
  //   {
  //     for (int i = 0; i < SimRobot.q.size(); i++)
  //     {
  //       RobotVelocityCur[i] = SimRobot.dq[i];
  //       if(RobotVelocityCur[i]<SimRobot.velMin(i))
  //       {
  //         RobotVelocityCur[i] = SimRobot.velMin(i);
  //       }
  //       if(RobotVelocityCur[i]>SimRobot.velMax(i))
  //       {
  //         RobotVelocityCur[i] = SimRobot.velMax(i);
  //       }
  //     }
  //   }
  //   break;
  //   default:
  //   {
  //     for (int i = 0; i < SimRobot.q.size(); i++)
  //     {
  //       double RobotVelocity_i = (qTrajAct[qTrajAct.size()-1][i] - qTrajAct[qTrajAct.size()-2][i])/dt;
  //       switch (i)
  //       {
  //         case 5:
  //         {
  //           if((RobotVelocity_i<-RollAngVelLimit)||(RobotVelocity_i>RollAngVelLimit))
  //           {
  //             RobotVelocityCur[i] = 0.0;
  //           }
  //           else
  //           {
  //             RobotVelocityCur[i] = RobotVelocity_i;
  //           }
  //         }
  //         break;
  //         case 10:
  //         {
  //           RobotVelocityCur[i] = SimRobot.dq[i];
  //         }
  //         break;
  //         case 11:
  //         {
  //           RobotVelocityCur[i] = SimRobot.dq[i];
  //         }
  //         break;
  //         case 16:
  //         {
  //           RobotVelocityCur[i] = SimRobot.dq[i];
  //         }
  //         break;
  //         case 17:
  //         {
  //           RobotVelocityCur[i] = SimRobot.dq[i];
  //         }
  //         break;
  //         default:
  //         {
  //           RobotVelocityCur[i] = RobotVelocity_i;
  //         }
  //         break;
  //       }
  //       if(RobotVelocityCur[i]<SimRobot.velMin(i))
  //       {
  //         RobotVelocityCur[i] = SimRobot.velMin(i);
  //       }
  //       if(RobotVelocityCur[i]>SimRobot.velMax(i))
  //       {
  //         RobotVelocityCur[i] = SimRobot.velMax(i);
  //       }
  //     }
  //   }
  //   break;
  // }

  Config RobotVelocityCurrent(RobotVelocityCur);
  qdotTrajAct.push_back(RobotVelocityCurrent);
  RobotVelocityRef = RobotVelocityCur;

  SimRobot.dq = RobotVelocityCurrent;

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
  // COMDes.x = 0.13563739414658971;
  // COMDes.y = 0.13093482618578417;
  // COMDes.z = 0.78386621712800796;

  COMVelDes.x = (COMDes.x - COMPos.x)/dt;
  COMVelDes.y = (COMDes.y - COMPos.y)/dt;
  COMVelDes.z = (COMDes.z - COMPos.z)/dt;

  _COMDes = COMDes;

  SimRobot.GetCOMJacobian(Jcom);

  std::vector<double> qNew;
  std::vector<double> qdotNew(DOF);

  _QPStatus = HJBVelocitySolver(SimRobot, dt, qdotNew);
  std::printf("QP status: %d\n", _QPStatus);

  double obj = 0.0;
  double Tracking = 0.0;
  for (int i = 0; i < 3; i++)
  {
    double CenVel_i = 0;
    for (int j = 0; j < DOF; j++)
    {
      CenVel_i = Jcom(i,j) * qdotNew[j];
    }
    Tracking+= (CenVel_i + Kcen * (COMPos[i] - COMDes[i])) * (CenVel_i + Kcen * (COMPos[i] - COMDes[i]));
  }

  // 1. Damping
  double Damping = 0.0;
  for (int i = 0; i < 3; i++)
  {
    double CenVel_i = 0;
    for (int j = 0; j < DOF; j++)
    {
      CenVel_i = Jcom(i,j) * qdotNew[j];
    }
    Damping+= Kdamp * CenVel_i * CenVel_i;
  }

  // 2. Regularization
  double Regularization = 0.0;
  for (int i = 0; i < DOF; i++)
  {
    Regularization+= Kreg * qdotNew[i] * qdotNew[i];
  }

  // 0. Centroidal Part
  Vector3 COMVelOpt(0.0, 0.0, 0.0);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < DOF; j++)
    {
      COMVelOpt[i]+=Jcom(i,j) * qdotNew[j];
    }
  }
  Vector3 TrackingCost = COMVelOpt + Kcen * (COMPos - COMDes);
  double TrackingCostVal = TrackingCost[0] * TrackingCost[0] + TrackingCost[1] * TrackingCost[1] + TrackingCost[2] * TrackingCost[2];
  std::printf("0. Tracking Cost: %f\n", TrackingCostVal);

  std::printf("1. Centroidal Damping Cost: %f\n", Kdamp * (COMVelOpt[0] * COMVelOpt[0] + COMVelOpt[1] * COMVelOpt[1] + COMVelOpt[2] * COMVelOpt[2]));
  double RegularizationCost = 0.0;
  for (int i = 0; i < DOF; i++)
  {
    RegularizationCost+= Kreg * qdotNew[i] * qdotNew[i];
  }
  std::printf("2. Regularization Cost: %f\n", RegularizationCost);

  std::vector<double> qDes(DOF), qdotDes(DOF), qddot(DOF);
  std::vector<double> qRef = qTraj[qTraj.size()-1];
  std::vector<double> qdotRef = qdotTraj[qdotTraj.size()-1];

  switch (_QPStatus)
  {
    case 1:
    {
      // This means that the current QP Controller indeed returns an optimal solution.
      qdotDes = qdotNew;
    }
    break;
    default:
    {
      for (int i = 0; i < DOF; i++)
      {
        qdotDes[i] = qdotRef[i];
      }
    }
    break;
  }

  // Now it is time to update the robot's commanded configuration.
  for (int i = 0; i < DOF; i++)
  {
    double qDes_i, qdotDes_i;
    qDes_i = qRef[i] + qdotDes[i] * dt;
    qdotDes_i = qdotDes[i];
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
