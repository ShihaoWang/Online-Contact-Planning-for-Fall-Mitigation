// This function is used to evaluate the feasibility of the current static stability
#include "NonlinearOptimizerInfo.h"
#include "RobotInfo.h"
#include "CommonHeader.h"
#include "gurobi_c++.h"

static double epsTol = 1e-8;
static double InfVal = 100000.0;
double CPSPGenerator(const std::vector<Vector3> & ActContacts, const Vector3 & COM, const Vector3 & COMVel, const double & mass, const std::vector<Vector3> & ConeUnits, const int & edge_no)
{
  // The variables to be optimized are force magnitudes.

  double CP_x_new = COM.x + COMVel.x/sqrt(9.81/COM.z);
  double CP_y_new = COM.y + COMVel.y/sqrt(9.81/COM.z);
  Vector3 COM_new(CP_x_new, CP_y_new, COM.z);
  double ConsVioVal = 1.0;

  // Number of Variables to be optimized and Constraints
  int n = ConeUnits.size();
  std::vector<double> ConeMagnitudes(n);
  int info = 0.0;
  try {
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);
    // Create variables
    std::vector<GRBVar> OptVariables;
    OptVariables.reserve(n);
    int VarInd = 0;
    for (int i = 0; i < n; i++)
    {
      std::string x_name = "x" + std::to_string(VarInd);
      GRBVar x_i = model.addVar(0.0, InfVal, 0.0, GRB_CONTINUOUS, x_name);
      OptVariables.push_back(x_i);
      VarInd += 1;
    }

    /* Set objective */
    GRBQuadExpr obj = 0;
    for (int i = 0; i < n; i++)
    {
      obj+=OptVariables[i] * OptVariables[i];
    }
    model.setObjective(obj);

    // Next step is to add constraint: there are two types of constraints
    // 0. Force Balance
    GRBLinExpr Dyn_fx_left_i= 0;
    GRBLinExpr Dyn_fy_left_i= 0;
    GRBLinExpr Dyn_fz_left_i= 0;

    GRBLinExpr Dyn_fx_right_i= 0;
    GRBLinExpr Dyn_fy_right_i= 0;
    GRBLinExpr Dyn_fz_right_i= mass * 9.81;

    for (int i = 0; i < n; i++)
    {
      Dyn_fx_left_i+=ConeUnits[i].x * OptVariables[i];
      Dyn_fy_left_i+=ConeUnits[i].y * OptVariables[i];
      Dyn_fz_left_i+=ConeUnits[i].z * OptVariables[i];
    }
    int ConsInd = 0;
    std::string cons_fx_name = "c" + std::to_string(ConsInd);
    model.addConstr(Dyn_fx_left_i==Dyn_fx_right_i, cons_fx_name);
    ConsInd+=1;
    std::string cons_fy_name = "c" + std::to_string(ConsInd);
    model.addConstr(Dyn_fy_left_i==Dyn_fy_right_i, cons_fy_name);
    ConsInd+=1;
    std::string cons_fz_name = "c" + std::to_string(ConsInd);
    model.addConstr(Dyn_fz_left_i==Dyn_fz_right_i, cons_fz_name);
    ConsInd+=1;

    // 1. Moment Balance
    GRBLinExpr Dyn_mx_left_i= 0;
    GRBLinExpr Dyn_my_left_i= 0;
    GRBLinExpr Dyn_mz_left_i= 0;

    GRBLinExpr Dyn_mx_right_i= 0;
    GRBLinExpr Dyn_my_right_i= 0;
    GRBLinExpr Dyn_mz_right_i= 0;
    for (int i = 0; i < n; i++)
    {
      int ContactIndex = floor(i/edge_no);
      Vector3 ContactPos = ActContacts[ContactIndex];
      Vector3 CoM2ContactPos = ContactPos - COM_new;
      Vector3 ConeUnitVec = ConeUnits[i] ;
      Vector3 MomentumCoeff_i = cross(CoM2ContactPos, ConeUnitVec);
      Dyn_mx_left_i+= OptVariables[i] * MomentumCoeff_i[0];
      Dyn_my_left_i+= OptVariables[i] * MomentumCoeff_i[1];
      Dyn_mz_left_i+= OptVariables[i] * MomentumCoeff_i[2];
    }
    std::string cons_mx_name = "c" + std::to_string(ConsInd);
    model.addConstr(Dyn_mx_left_i==Dyn_mx_right_i, cons_fx_name);
    ConsInd+=1;
    std::string cons_my_name = "c" + std::to_string(ConsInd);
    model.addConstr(Dyn_my_left_i==Dyn_my_right_i, cons_fy_name);
    ConsInd+=1;
    std::string cons_mz_name = "c" + std::to_string(ConsInd);
    model.addConstr(Dyn_mz_left_i==Dyn_mz_right_i, cons_fz_name);
    ConsInd+=1;

    model.optimize();
    // cout << "Capture Point with Support Polygon Objective Value: " << model.get(GRB_DoubleAttr_ObjVal)<< endl;

    for (int i = 0; i < n; i++)
    {
      ConeMagnitudes[i] = OptVariables[i].get(GRB_DoubleAttr_X);
    }

    std::vector<Vector3> OptContactForce;
    OptContactForce.reserve(ConeUnits.size()/edge_no);

    int OptIndex = 0;
    for (int i = 0; i < ConeUnits.size()/edge_no; i++)
    {
      Vector3 OptContactForce_i(0.0, 0.0, 0.0);
      for (int j = 0; j < edge_no; j++)
      {
        OptContactForce_i+=ConeUnits[OptIndex] * ConeMagnitudes[OptIndex];
        OptIndex++;
      }
      OptContactForce.push_back(OptContactForce_i);
    }

    std::vector<double> ConstraintVec(6);
    // Force Balance
    double F_x = 0.0;
    double F_y = 0.0;
    double F_z = 0.0;
    for (int i = 0; i < ConeUnits.size(); i++)
    {
      F_x+= ConeUnits[i].x * ConeMagnitudes[i];
      F_y+= ConeUnits[i].y * ConeMagnitudes[i];
      F_z+= ConeUnits[i].z * ConeMagnitudes[i];
    }
    ConstraintVec[0] = F_x * F_x;
    ConstraintVec[1] = F_y * F_y;
    ConstraintVec[2] = (F_z - mass * 9.81) * (F_z - mass * 9.81);

    // Momentum Balance
    double M_x = 0.0;
    double M_y = 0.0;
    double M_z = 0.0;
    for (int i = 0; i < ConeUnits.size(); i++)
    {
      int ContactIndex = floor(i/edge_no);
      Vector3 ContactPos = ActContacts[ContactIndex];
      Vector3 CoM2ContactPos = ContactPos - COM_new;
      Vector3 ConeUnitVec = ConeUnits[i] * ConeMagnitudes[i];
      Vector3 MomentumCoeff_i = cross(CoM2ContactPos, ConeUnitVec);

      M_x+= MomentumCoeff_i.x;
      M_y+= MomentumCoeff_i.y;
      M_z+= MomentumCoeff_i.z;
    }
    ConstraintVec[3] = M_x * M_x;
    ConstraintVec[4] = M_y * M_y;
    ConstraintVec[5] = M_z * M_z;

    ConsVioVal = *std::max_element(ConstraintVec.begin(), ConstraintVec.end());

  } catch(GRBException e)
  {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
    return 1.0;
  } catch(...)
  {
    cout << "Exception during optimization" << endl;
    return 1.0;
  }
  if(ConsVioVal<epsTol)
  {
    info = 0.0;
  }
  else
  {
    info = 1.0;
  }
  return info;
}
