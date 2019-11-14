#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"

int EndEffectorFixer(Robot & SimRobot, const PIPInfo & PIPObj, const std::vector<LinkInfo> & RobotLinkInfo, const std::vector<ContactStatusInfo> & RobotContactInfo, SignedDistanceFieldInfo & SDFInfo, const Meshing::PointCloud3D & PointCloudObj)
{
  // This function is used to fix the end effector whose contact should not be modified.
  // First the job is to figure out whether this edge belongs to a certain end effector

  // Evaluation of the current contact status for contact addition consideration.
  std::vector<int> ContactAddInfoIndices;
  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    switch (RobotContactInfo[i].LocalContactStatus[0])
    {
      case 0:
      {
        // This indicates that there are potentially available end effector to make contact.
        ContactAddInfoIndices.push_back(i);
      }
      break;
      default:
      break;
    }
  }

  // This part is to figure out what contact can be changed.
  std::vector<Vector3> ModiCOMPosTraj, ModiCOMVelTraj;
  int FixedContactIndex;
  double ContactModiColTime = ContactModiPreEstimation(SimRobot, PIPObj, RobotLinkInfo, RobotContactInfo, SDFInfo, FixedContactIndex, ModiCOMPosTraj, ModiCOMVelTraj);
  std::vector<int> ContactModiInfoIndices;
  for (int i = 0; i < RobotLinkInfo.size(); i++)
  {
    switch (RobotContactInfo[i].LocalContactStatus[0])
    {
      case 1:
      {
        // This indicates that there are potentially available end effector(s) to make contact.
        if(i == FixedContactIndex)
        {

        }
        else
        {
          ContactModiInfoIndices.push_back(i);
        }
      }
      break;
      default:
      break;
    }
  }

  switch (ContactAddInfoIndices.size())
  {
    case 0:
    {
      // This means that all current contacts are active so no additional contact can be added!
    }
    break;
    default:
    {

      std::vector<Vector3> AddCOMPosTraj, AddCOMVelTraj;
      double ContactAddColTime = ContactAddPreEstimation(SimRobot, PIPObj, SDFInfo, AddCOMPosTraj, AddCOMVelTraj);

    }
    break;
  }

  int a = 1;
  return 1;
}
