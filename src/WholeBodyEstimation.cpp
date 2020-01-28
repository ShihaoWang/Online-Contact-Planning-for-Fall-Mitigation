#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"
#include <omp.h>
#include "NonlinearOptimizerInfo.h"

/*
    This function computes the robot's whole-body configuration based on the inverted pendulum rotation motion around axis.
*/

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
static Vector3 RotMat2EulerAngles(Matrix3 & RotMat)
{
  float sy = sqrt(RotMat(0,0) * RotMat(0,0) +  RotMat(1,0) * RotMat(1,0) );

  bool singular = sy < 1e-6; // If

  float x, y, z;
  if (!singular)
  {
    x = atan2(RotMat(2,1) , RotMat(2,2));
    y = atan2(-RotMat(2,0), sy);
    z = atan2(RotMat(1,0), RotMat(0,0));
  }
  else
  {
    x = atan2(-RotMat(1,2), RotMat(1,1));
    y = atan2(-RotMat(2,0), sy);
    z = 0;
  }
  return Vector3(x, y, z);
}

static Vector3 RigidBodyRotation(const Vector3 & RigidBodyPoint, const double & RotAngle, const Vector3 & RotAxis, const Vector3 & AxisOri)
{
  // This function is used to calcualte the new RigidBodyPoint after the rotation around EdgeA->EdgeB axis for Angle degree.
  AngleAxisRotation RotMatrix(RotAngle, RotAxis);
  Vector3 RigidBodyPointNew;
  RotMatrix.transformPoint(RigidBodyPoint - AxisOri, RigidBodyPointNew);
  RigidBodyPointNew+=AxisOri;
  return RigidBodyPointNew;
}

static void StepIntegrator(InvertedPendulumInfo & InvertedPendulumObj, const PIPInfo & PIPObj, const Vector3 & RotAxis, const double & TimeStep)
{
  // This function is used to integrate robot's PIP dynamics.
  double L = PIPObj.L;
  double g = PIPObj.g;

  // Integration with the assumption that robot's acceleration remains to be constant during TimeStep.
  double Thetaddot = g/L * sin(InvertedPendulumObj.Theta);
  double ThetaOffset = InvertedPendulumObj.Thetadot * TimeStep + 0.5 * Thetaddot * TimeStep * TimeStep;
  // double ThetaOffset = InvertedPendulumObj.Thetadot * TimeStep;
  double ThetadotOffset = Thetaddot * TimeStep;

  double ThetaNew = InvertedPendulumObj.Theta + ThetaOffset;
  double ThetadotNew = InvertedPendulumObj.Thetadot + ThetadotOffset;

  Vector3 COMPosNew = RigidBodyRotation(InvertedPendulumObj.COMPos, -ThetaOffset, RotAxis, PIPObj.EdgeA);
  Vector3 COMPosOnEdge = PIPObj.EdgeA + RotAxis.dot(COMPosNew - PIPObj.EdgeA) * RotAxis;
  Vector3 COMVelDir;
  COMVelDir.setNormalized(cross(COMPosNew - COMPosOnEdge, RotAxis));
  Vector3 COMVelNew = L * ThetadotNew * COMVelDir;

  InvertedPendulumObj.Theta =  ThetaNew;
  InvertedPendulumObj.Thetadot =  ThetadotNew;
  InvertedPendulumObj.COMPos =  COMPosNew;
  InvertedPendulumObj.COMVel =  COMVelNew;
  return;
}

static std::vector<double> GlobalFrameConfigUpdate(Robot & SimRobot, const double & ThetaOffset, const Vector3 & RotAxis, const Vector3 & AxisOri)
{
  // This function is used to update robot's configuration for global frame..
  // First part is frame's Euclidean position.

  // std::cout<<SimRobot.q<<endl;
  Vector3 FramePos, FramePos1;
  SimRobot.GetWorldPosition(Vector3(0.0, 0.0, 0.0), 0, FramePos);   // This gets robot's position of global frame.
  SimRobot.GetWorldPosition(Vector3(0.0, 0.0, 0.0), 1, FramePos);   // This gets robot's position of global frame
  // SimRobot.GetWorldPosition(Vector3(0.0, 0.0, -0.708), 2, FramePos);   // This gets robot's position of global frame.
  SimRobot.GetWorldPosition(Vector3(0.0, 0.0, 0.0), 2, FramePos);   // This gets robot's position of global frame.

  Vector3 FrameOri, FrameXaxis, FrameYaxis, FrameZaxis;
  SimRobot.GetWorldPosition(Vector3(0.0, 0.0, 0.0), 5, FrameOri);       // This gets robot's position of global frame.
  SimRobot.GetWorldPosition(Vector3(0.0, 0.0, 1.0), 5, FrameXaxis);
  SimRobot.GetWorldPosition(Vector3(0.0, 1.0, 0.0), 5, FrameYaxis);
  SimRobot.GetWorldPosition(Vector3(1.0, 0.0, 0.0), 5, FrameZaxis);

  Vector3 FramePosNew = RigidBodyRotation(FramePos, -ThetaOffset, RotAxis, AxisOri);

  Vector3 x_axis, y_axis, z_axis;
  x_axis = FrameXaxis - FrameOri;
  y_axis = FrameYaxis - FrameOri;
  z_axis = FrameZaxis - FrameOri;
  x_axis.setNormalized(x_axis);
  y_axis.setNormalized(y_axis);
  z_axis.setNormalized(z_axis);

  Matrix3 RotMat(z_axis, y_axis, x_axis);
  AngleAxisRotation RotMatrix(-ThetaOffset, RotAxis);
  Matrix3 NewRotMat;
  RotMatrix.getMatrix(NewRotMat);

  NewRotMat.mul(NewRotMat, RotMat);
  Vector3 EulerAngle = RotMat2EulerAngles(NewRotMat);    // Reverse order to update frame's yaw, pitch and roll.
  double Pi = atan(1.0)*4.0;
  if(EulerAngle.x>Pi)
  {
    EulerAngle.x-=2.0 * Pi;
  }
  if(EulerAngle.x<-Pi)
  {
    EulerAngle.x+=2.0 * Pi;
  }
  double FrameConfig[] = {FramePosNew.x, FramePosNew.y, FramePosNew.z, EulerAngle.z , EulerAngle.y, EulerAngle.x};
  std:;vector<double> FrameConfigVec(FrameConfig, FrameConfig + 6);
  return FrameConfigVec;
}

Config WholeBodyDynamicsIntegrator(Robot & SimRobot, const std::vector<double> & _OptConfig, const PIPInfo & PIPObj, InvertedPendulumInfo & InvertedPendulumObj, const double & TimeDuration, const int & StepIndex)
{
  // This function is used to update robot's whole-body configuration based on inverted pendulum model
  // SimRobot should have already been updated with the previous optimized configuration.
  Vector3 RotAxis = PIPObj.EdgeB - PIPObj.EdgeA;
  RotAxis.setNormalized(RotAxis);

  const int IntergrationStep = 11;
  int IntergrationIndex = 0;
  double TimeStep = TimeDuration/(1.0 * IntergrationStep - 1.0);
  double ThetaInit = InvertedPendulumObj.Theta;
  while(IntergrationIndex<IntergrationStep)
  {
    StepIntegrator(InvertedPendulumObj, PIPObj, RotAxis, TimeStep);
    IntergrationIndex++;
  }
  // Here the integration has been finished.
  // Let's compute the robot's new configuration.
  double ThetaOffset = InvertedPendulumObj.Theta - ThetaInit;

  // The most interesting part is as follows.
  // std::cout<<SimRobot.q<<endl;
  std::vector<double> UpdateConfig = GlobalFrameConfigUpdate(SimRobot, ThetaOffset, RotAxis, PIPObj.EdgeA);
  std::string ConfigPath = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/user/hrp2/";
  string _OptConfigFile = "OptConfig" + std::to_string(StepIndex) + ".config";
  RobotConfigWriter(_OptConfig, ConfigPath, _OptConfigFile);
  std::vector<double> OptConfig = _OptConfig;

  OptConfig[0] = UpdateConfig[0];
  OptConfig[1] = UpdateConfig[1];
  OptConfig[2] = UpdateConfig[2];
  OptConfig[3] = UpdateConfig[3];
  OptConfig[4] = UpdateConfig[4];
  OptConfig[5] = UpdateConfig[5];

  return Config(OptConfig);
}
