#include "NonlinearOptimizerInfo.h"
#include "CommonHeader.h"
#include <random>
#include "gurobi_c++.h"

// This function is used to compute the stabilizing controller to regain balance for the disturbed robot.

static double Kd = 1.0;
static double InfVal = 100.0;

std::vector<double> ContactController(const Robot & SimRobot, EndPathInfo & EndSplineObj, const double& dt, std::vector<Config>& qTrajDes, std::vector<Config> & qdotTrajDes, std::vector<Config> & qTrajAct, std::vector<Config> & qdotTrajAct, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & _RobotContactInfo)
{
  // This function solves for the delta_s and delta_q such that travels along the path as fast as possible.
  // variable to be minimized: delta_s and delta_q : 1 + DOF
  int DOF = SimRobot.q.size();
  double VelLimit = 3.0;
  qTrajAct.push_back(SimRobot.q);
  std::vector<double> RobotVelocityCur(DOF);
  // Velocity estimation with finite-difference.
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
  for (int i = 6; i < SimRobot.q.size(); i++)
  {
    double RobotVelocity_i = (qTrajAct[qTrajAct.size()-1][i] - qTrajAct[qTrajAct.size()-2][i])/dt;
    switch (i)
    {
      case 10:
      {
        RobotVelocityCur[i] = SimRobot.dq[i];
      }
      break;
      case 11:
      {
        RobotVelocityCur[i] = SimRobot.dq[i];
      }
      break;
      case 16:
      {
        RobotVelocityCur[i] = SimRobot.dq[i];
      }
      break;
      case 17:
      {
        RobotVelocityCur[i] = SimRobot.dq[i];
      }
      break;
      default:
      {
        RobotVelocityCur[i] = RobotVelocity_i;
      }
      break;
    }
    if(RobotVelocityCur[i]<SimRobot.velMin(i))
    {
      RobotVelocityCur[i] = SimRobot.velMin(i);
    }
    if(RobotVelocityCur[i]>SimRobot.velMax(i))
    {
      RobotVelocityCur[i] = SimRobot.velMax(i);
    }
  }
  Config RobotVelocityCurrent(RobotVelocityCur);
  qdotTrajAct.push_back(RobotVelocityCurrent);

  std::vector<double> qDes(DOF), qdotDes(DOF);
  std::vector<double> qRef = qTrajDes[qTrajDes.size()-1];
  std::vector<double> qdotRef = qdotTrajDes[qdotTrajDes.size()-1];

  // If robot's current kinetic energy is too small, there is no need to conduct LP.
  double KEtol = 0.001;
  double KE_i = SimRobot.GetKineticEnergy();
  if(KE_i<=KEtol)
  {
    return qRef;
  }

  /*
    Here is the linear programming procedure.
  */

  // Jacobian matrices are twofolds: fixed contact and modified contact.
  std::vector<ContactStatusInfo> RobotContactInfo = _RobotContactInfo;
  switch (RobotContactInfo[EndSplineObj.EndEffectorIndex].LocalContactStatus[0])
  {
    case 1:
    {
      // This indicates that assumed contact status is active.
      for (int i = 0; i < RobotContactInfo[EndSplineObj.EndEffectorIndex].LocalContactStatus.size(); i++)
      {
        RobotContactInfo[EndSplineObj.EndEffectorIndex].LocalContactStatus[i] = 0;
      }
    }
    break;
    default:
    {
    }
    break;
  }

  int FixedContactNumber = 0;
  for (int i = 0; i < RobotContactInfo.size(); i++)
  {
    for (int j = 0; j < RobotContactInfo[i].LocalContactStatus.size(); j++)
    {
      switch (RobotContactInfo[i].LocalContactStatus[j])
      {
        case 1:
        {
          FixedContactNumber++;
        }
        break;
        default:
        break;
      }
    }
  }

  std::vector<Matrix> FixedJac;
  FixedJac.reserve(FixedContactNumber);
  for (int i = 0; i < RobotContactInfo.size(); i++)
  {
    switch (RobotContactInfo[i].LocalContactStatus[0])
    {
      case 1:
      {
        for (int j = 0; j < RobotContactInfo[i].LocalContactStatus.size(); j++)
        {
          Matrix ActJacobian;
          SimRobot.GetPositionJacobian(RobotLinkInfo[i].LocalContacts[j], RobotLinkInfo[i].LinkIndex, ActJacobian);
          FixedJac.push_back(ActJacobian);
        }
      }
      break;
      default:
      {
      }
      break;
    }
  }
  Vector3 PosMod;
  Matrix  JacMod;
  SimRobot.GetWorldPosition(RobotLinkInfo[EndSplineObj.EndEffectorIndex].AvgLocalContact,     RobotLinkInfo[EndSplineObj.EndEffectorIndex].LinkIndex,   PosMod);
  SimRobot.GetPositionJacobian(RobotLinkInfo[EndSplineObj.EndEffectorIndex].AvgLocalContact,  RobotLinkInfo[EndSplineObj.EndEffectorIndex].LinkIndex,   JacMod);


  int n = 1 + DOF;
  int neF = 0;                                  // Objective
  neF = neF + DOF + DOF;                        // Velocity + Acceleration
  neF = neF + 3 ;                               // Task Contact Velocity.
  neF = neF + FixedJac.size() * 3 ;             // Fixed Contact Velocity.

  std::vector<double> x_soln(n);
  int info;

  double sCur = EndSplineObj.Pos2s(PosMod);

  try {
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);
    // Create variables
    std::vector<GRBVar> OptVariables;
    OptVariables.reserve(n);

    int VarInd = 0;

    // 0.delta_s
    for (int i = 0; i < 1; i++)
    {
      std::string x_name = "x" + std::to_string(VarInd);
      double sUpper = 1.0 - sCur;
      GRBVar x_i = model.addVar(0.0, sUpper, 0.0, GRB_CONTINUOUS, x_name);
      OptVariables.push_back(x_i);
      VarInd += 1;
    }

    // 1. delta_q: x,y,z,y,p,r
    for (int i = 1; i < 7; i++)
    {
      std::string x_name = "x" + std::to_string(VarInd);
      GRBVar x_i = model.addVar(-1.0 * InfVal, InfVal, 0.0, GRB_CONTINUOUS, x_name);
      OptVariables.push_back(x_i);
      VarInd += 1;
    }
    // 1. delta_q: actuated joints
    for (int i = 7; i < DOF + 1; i++)
    {
      std::string x_name = "x" + std::to_string(VarInd);
      double delta_q_min = SimRobot.qMin(i-1) - qRef[i-1];
      double delta_q_max = SimRobot.qMax(i-1) - qRef[i-1];
      GRBVar x_i = model.addVar(delta_q_min, delta_q_max, 0.0, GRB_CONTINUOUS, x_name);
      OptVariables.push_back(x_i);
      VarInd += 1;
    }

    /*
      Set objective
    */
    GRBLinExpr obj = -OptVariables[0];    //Maximize s to be as fast as possible.
    model.setObjective(obj);

    /*
      Set constraints
    */
    int ConsInd = 0;

    // 1. Velocity left bounds
    for (int i = 0; i < 6; i++)
    {
      GRBLinExpr ConAcc_left_i  = -InfVal;
      GRBLinExpr ConAcc_right_i = OptVariables[i+1]/dt;
      std::string cons_name = "c" + std::to_string(ConsInd);
      model.addConstr(ConAcc_left_i<=ConAcc_right_i, cons_name);
      ConsInd+=1;
    }
    for (int i = 6; i < DOF; i++)
    {
      GRBLinExpr ConAcc_left_i  = SimRobot.velMin(i);
      GRBLinExpr ConAcc_right_i = OptVariables[i+1]/dt;
      std::string cons_name = "c" + std::to_string(ConsInd);
      model.addConstr(ConAcc_left_i<=ConAcc_right_i, cons_name);
      ConsInd+=1;
    }
    // 1. Velocity right bounds
    for (int i = 0; i < 6; i++)
    {
      GRBLinExpr ConAcc_left_i  = OptVariables[i+1]/dt;
      GRBLinExpr ConAcc_right_i = InfVal;
      std::string cons_name = "c" + std::to_string(ConsInd);
      model.addConstr(ConAcc_left_i<=ConAcc_right_i, cons_name);
      ConsInd+=1;
    }
    for (int i = 6; i < DOF; i++)
    {
      GRBLinExpr ConAcc_left_i  = OptVariables[i+1]/dt;
      GRBLinExpr ConAcc_right_i = SimRobot.velMax(i);
      std::string cons_name = "c" + std::to_string(ConsInd);
      model.addConstr(ConAcc_left_i<=ConAcc_right_i, cons_name);
      ConsInd+=1;
    }

    // 2. Acceleration left bounds
    for (int i = 0; i < 6; i++)
    {
      GRBLinExpr ConAcc_left_i  = -InfVal;
      GRBLinExpr ConAcc_right_i = OptVariables[i+1]/(dt * dt) - RobotVelocityCur[i]/dt;
      std::string cons_name = "c" + std::to_string(ConsInd);
      model.addConstr(ConAcc_left_i<=ConAcc_right_i, cons_name);
      ConsInd+=1;
    }
    for (int i = 6; i < DOF; i++)
    {
      GRBLinExpr ConAcc_left_i  = -SimRobot.accMax(i);
      GRBLinExpr ConAcc_right_i = OptVariables[i+1]/(dt * dt) - RobotVelocityCur[i]/dt;
      std::string cons_name = "c" + std::to_string(ConsInd);
      model.addConstr(ConAcc_left_i<=ConAcc_right_i, cons_name);
      ConsInd+=1;
    }
    // 2. Acceleration right bounds
    for (int i = 0; i < DOF; i++)
    {
      GRBLinExpr ConAcc_left_i  = OptVariables[i+1]/(dt * dt) - RobotVelocityCur[i]/dt;
      GRBLinExpr ConAcc_right_i = InfVal;
      std::string cons_name = "c" + std::to_string(ConsInd);
      model.addConstr(ConAcc_left_i<=ConAcc_right_i, cons_name);
      ConsInd+=1;
    }
    for (int i = 6; i < DOF; i++)
    {
      GRBLinExpr ConAcc_left_i  = OptVariables[i+1]/(dt * dt) - RobotVelocityCur[i]/dt;
      GRBLinExpr ConAcc_right_i = SimRobot.accMax(i);
      std::string cons_name = "c" + std::to_string(ConsInd);
      model.addConstr(ConAcc_left_i<=ConAcc_right_i, cons_name);
      ConsInd+=1;
    }
    // 3. Task Contact velocity constraint
    Vector3 ContactPos, ContactVelocity;
    EndSplineObj.PosNTang(sCur, ContactPos, ContactVelocity);
    ContactVelocity+=Kd * (EndSplineObj.GoalContactPos - PosMod);     // Served as a feedforward term.
    for (int i = 0; i < 3; i++)
    {
      GRBLinExpr ConAcc_left_i  = OptVariables[0]*ContactVelocity[i];
      GRBLinExpr ConAcc_right_i = 0.0;
      for (int j = 0; j < DOF; j++)
      {
        ConAcc_right_i+=JacMod(i,j) * OptVariables[j+1];
      }
      std::string cons_name = "c" + std::to_string(ConsInd);
      model.addConstr(ConAcc_left_i==ConAcc_right_i, cons_name);
      ConsInd+=1;
    }

    // 4. Fixed Contact Velocity Constraint
    for (int i = 0; i < FixedJac.size(); i++)
    {
      for (int j = 0; j < 3; j++)
      {
        GRBLinExpr ConAcc_left_i  = 0.0;
        GRBLinExpr ConAcc_right_i = 0.0;
        for (int k = 0; k < DOF; k++)
        {
          ConAcc_left_i+=FixedJac[i](j,k) * OptVariables[k+1];
        }
        std::string cons_name = "c" + std::to_string(ConsInd);
        model.addConstr(ConAcc_left_i==ConAcc_right_i, cons_name);
        ConsInd+=1;
      }
    }
    model.optimize();
    cout << "Objective Value: " << model.get(GRB_DoubleAttr_ObjVal)<< endl;
    for (int i = 0; i < n; i++)
    {
      x_soln[i] = OptVariables[i].get(GRB_DoubleAttr_X);
    }
  } catch(GRBException e)
  {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...)
  {
    cout << "Exception during optimization" << endl;
  }

  for (int i = 0; i < DOF; i++)
  {
    double qDes_i = qRef[i] + x_soln[i + 1];
    double qdotDes_i = qdotRef[i] + x_soln[i + 1]/dt;
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
  qTrajDes.push_back(qDes_);
  qdotTrajDes.push_back(qdotDes_);

  return qDes;
}
