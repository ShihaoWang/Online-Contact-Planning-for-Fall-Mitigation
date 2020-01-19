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

static Vector3 RigidBodyRotation(const Vector3 & RigidBodyPoint, const double & RotAngle, const Vector3 & RotAxis)
{
  // This function is used to calcualte the new RigidBodyPoint after the rotation around EdgeA->EdgeB axis for Angle degree.
  AngleAxisRotation RotMatrix(RotAngle, RotAxis);
  Vector3 RigidBodyPointNew;
  RotMatrix.transformPoint(RigidBodyPoint, RigidBodyPointNew);
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
  double ThetadotOffset = Thetaddot * TimeStep;

  double ThetaNew = InvertedPendulumObj.Theta + ThetaOffset;
  double ThetadotNew = InvertedPendulumObj.Thetadot + ThetadotOffset;

  Vector3 COMPosNew = RigidBodyRotation(InvertedPendulumObj.COMPos, ThetaOffset, RotAxis);
  Vector3 COMPosOnEdge = PIPObj.EdgeA + RotAxis.dot(COMPosNew - PIPObj.EdgeA) * RotAxis;
  Vector3 COMVelDir;
  COMVelDir.setNormalized(cross(COMPosNew - COMPosOnEdge, RotAxis));
  Vector3 COMVelNew = ThetadotNew * COMVelDir;

  InvertedPendulumObj.Theta =  ThetaNew;
  InvertedPendulumObj.Thetadot =  ThetadotNew;
  InvertedPendulumObj.COMPos =  COMPosNew;
  InvertedPendulumObj.COMVel =  COMVelNew;
  return;
}

Config WholeBodyDynamicsIntegrator(Robot & SimRobot, const PIPInfo & PIPObj, InvertedPendulumInfo & InvertedPendulumObj, const double & TimeDuration)
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
  std::cout<<SimRobot.q<<endl;
  Vector3 FramePos1, FramePos2, FramePos3;
  Vector3 FramePos4, FramePos5, FramePos6;
  Vector3 FramePosO;
  SimRobot.GetWorldPosition(Vector3(0.0, 0.0, -0.708), 0, FramePos1);
  SimRobot.GetWorldPosition(Vector3(0.0, 0.0, -0.708), 1, FramePos2);
  SimRobot.GetWorldPosition(Vector3(0.0, 0.0, -0.708), 2, FramePos3);   // This gets robot's position of global frame.

  Vector3 FramePos3New = RigidBodyRotation(FramePos3, ThetaOffset, RotAxis);

  SimRobot.GetWorldPosition(Vector3(0.0, 0.0, 0.0), 5, FramePosO);   // This gets robot's position of global frame.

  SimRobot.GetWorldPosition(Vector3(0.0, 0.0, 1.0), 5, FramePos4);
  SimRobot.GetWorldPosition(Vector3(0.0, 1.0, 0.0), 5, FramePos5);
  SimRobot.GetWorldPosition(Vector3(1.0, 0.0, 0.0), 5, FramePos6);   // This gets robot's position of global frame.

  Vector3 x_axis, y_axis, z_axis;
  x_axis = FramePos4 - FramePosO;
  y_axis = FramePos5 - FramePosO;
  z_axis = FramePos6 - FramePosO;

  x_axis.setNormalized(x_axis);
  y_axis.setNormalized(y_axis);
  z_axis.setNormalized(z_axis);

  std::cout<<x_axis<<endl;
  std::cout<<y_axis<<endl;
  std::cout<<z_axis<<endl;

  // Matrix3 RotMat(x_axis, y_axis, z_axis);
  Matrix3 RotMat(z_axis, y_axis, x_axis);

  Vector3 EulerAngle = RotMat2EulerAngles(RotMat);    // Reverse order to update frame's yaw, pitch and roll.

  std::vector<double> UpdateConfig;

  std::string ConfigPath = "/home/motion/Desktop/Online-Contact-Planning-for-Fall-Mitigation/user/hrp2/";

  RobotConfigWriter(UpdateConfig, ConfigPath, "UpdateConfig.config");

  std::cout<<endl;
}
