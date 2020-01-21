#include "NonlinearOptimizerInfo.h"
#include "CommonHeader.h"
#include "gurobi_c++.h"

// This function is used to estimate robot's collision impulse.

static double InfVal = 100000.0;

static void JacobianMatrixStack(const std::vector<Matrix> & ActJacs, Matrix & Jac, Matrix & JacTrans)
{
  // This function is used to stack Jacobian matrices into a single matrix.
  int RowNo = ActJacs.size() * ActJacs[0].m;
  int ColNo = ActJacs[0].n;
  Jac.resize(RowNo, ColNo);
  for (int i = 0; i < ActJacs.size(); i++)
  {
    int RowIndex = i * ActJacs[0].m;
    for(int j = 0; j < ActJacs[0].n; j++)
    {
      Jac(RowIndex + i,j) = ActJacs[i](i,j);
    }
  }
  JacTrans.setTranspose(Jac);
  return;
}

void CollisionImpulseFunc(Robot& SimRobot, const std::vector<ContactStatusInfo> & InitContactInfo, const int & SwingLimbIndex, ControlReferenceInfo & ControlReferenceObj)
{
  double Impulse = 0.0;
  std::vector<ContactStatusInfo> GoalContactInfo = InitContactInfo;
  for(int i = 0; i<InitContactInfo[SwingLimbIndex].LocalContactStatus.size(); i++)
  {
    GoalContactInfo[SwingLimbIndex].LocalContactStatus[i] = 1;
  }
  ControlReferenceObj.InitContactInfo = InitContactInfo;
  ControlReferenceObj.GoalContactInfo = GoalContactInfo;

  // Then get the active Jacobian matrices.
  std::vector<Matrix> ActJacs;
  for (int i = 0; i < GoalContactInfo.size(); i++)
  {
    switch (GoalContactInfo[i].LocalContactStatus[0])
    {
      case 1:
      {
        Matrix ActJac;
        SimRobot.GetPositionJacobian(NonlinearOptimizerInfo::RobotLinkInfo[i].AvgLocalContact, GoalContactInfo[i].LinkIndex, ActJac);
        ActJacs.push_back(ActJac);
      }
      break;
      default:
      break;
    }
  }

  Matrix Jac, JacTrans;
  JacobianMatrixStack(ActJacs, Jac, JacTrans);


  const int DOF = ControlReferenceObj.ConfigTraj[0].size();
  const int ConfigSize = ControlReferenceObj.ConfigTraj.size();

  Config Config1stLast = ControlReferenceObj.ConfigTraj[ConfigSize-1];
  Config Config2ndLast = ControlReferenceObj.ConfigTraj[ConfigSize-2];
  double TimeDuration = ControlReferenceObj.TimeTraj[ConfigSize-1] - ControlReferenceObj.TimeTraj[ConfigSize-2];

  NewtonEulerSolver NESolver(SimRobot);

  std::vector<double> qdotminus(DOF);
  for (int i = 0; i < DOF; i++)
  {
    qdotminus[i] = (Config1stLast[i] - Config2ndLast[i])/TimeDuration;
  }
  Vector qdotpre(qdotminus);

  Vector Jqdotpre;
  Jac.mul(qdotpre, Jqdotpre);

  Matrix DinvJTrans;
  NESolver.MulKineticEnergyMatrixInverse(JacTrans, DinvJTrans);

  Matrix JDinvJTrans;
  JDinvJTrans.mul(Jac, DinvJTrans);

  try {
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    // Create variables
    std::vector<GRBVar> OptVariables;
    OptVariables.reserve(Jac.m);
    GRBQuadExpr obj;
    int VarInd = 0;
    for (int i = 0; i < Jac.m; i++)
    {
      std::string x_name = "x" + std::to_string(VarInd);
      GRBVar x_i = model.addVar(-1.0 * InfVal, InfVal, 0.0, GRB_CONTINUOUS, x_name);
      obj+=x_i * x_i;
      OptVariables.push_back(x_i);
      VarInd += 1;
    }
    // Set objective
    model.setObjective(obj);
    int ConsInd = 0;
    for(int i = 0; i< Jac.m; i++)
    {
      std::string cons_name = "c" + std::to_string(ConsInd);
      GRBLinExpr Dyn_left_i = 0;
      for(int j = 0; j < Jac.m; j++)
      {
        Dyn_left_i+=JDinvJTrans(i,j) * OptVariables[j];
      }
      GRBLinExpr Dyn_right_i = -Jqdotpre[i];
      model.addConstr(Dyn_left_i==Dyn_right_i, cons_name);
      ConsInd+=1;
    }

    // Optimize model
    model.optimize();
    cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
    Impulse = sqrt(model.get(GRB_DoubleAttr_ObjVal));
  }
  catch(GRBException e)
  {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  }
  catch(...)
  {
    cout << "Exception during optimization" << endl;
  }

  ControlReferenceObj.Impulse = Impulse;
  return;
}
