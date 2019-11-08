// This function is used to load in the HJB data for the fall detection and contact planning.
#include "RobotInfo.h"

ViabilityKernelInfo ViabilityKernelDataLoader(const string & FailureMetricPath, const bool & FastFlag)
{
  const std::string HJBDataSpecsFilePath = FailureMetricPath + "PVKDataSpecs.bin";
  const char * HJBDataSpecsFilePathChar = HJBDataSpecsFilePath.c_str();
  FILE* HJBDataSpecsFile = fopen(HJBDataSpecsFilePathChar, "rb");
  std::vector<double> HJBDataSpecsVector(16);
  fread(&HJBDataSpecsVector[0], sizeof(double), HJBDataSpecsVector.size(), HJBDataSpecsFile);
  fclose(HJBDataSpecsFile);

  double LLow = HJBDataSpecsVector[0];
  double LUpp = HJBDataSpecsVector[1];
  double LdotLow = HJBDataSpecsVector[2];
  double LdotUpp = HJBDataSpecsVector[3];
  double ThetaLow = HJBDataSpecsVector[4];
  double ThetaUpp = HJBDataSpecsVector[5];
  double ThetadotLow = HJBDataSpecsVector[6];
  double ThetadotUpp = HJBDataSpecsVector[7];

  const int L_Grids = round(HJBDataSpecsVector[8]);
  const int Ldot_Grids = round(HJBDataSpecsVector[9]);
  const int Theta_Grids = round(HJBDataSpecsVector[10]);
  const int Thetadot_Grids = round(HJBDataSpecsVector[11]);

  double L_unit = (LUpp - LLow)/(1.0 *L_Grids - 1.0);
  double Ldot_unit = (LdotUpp - LdotLow)/(1.0 *Ldot_Grids - 1.0);
  double Theta_unit = (ThetaUpp - ThetaLow)/(1.0 *Theta_Grids - 1.0);
  double Thetadot_unit = (ThetadotUpp - ThetadotLow)/(1.0 *Thetadot_Grids - 1.0);

  int AngleLow = round(HJBDataSpecsVector[12]);
  int AngleUpp = round(HJBDataSpecsVector[13]);
  int AngleDiff = round(HJBDataSpecsVector[14]);
  double DeltaT = HJBDataSpecsVector[15];

  // ViabilityKernelInfo initialization
  ViabilityKernelInfo VKObj(HJBDataSpecsVector);

  switch (FastFlag)
  {
    case true:
    {
      return VKObj;
    }
    default:
    {
      std::printf("Load in Polytopic Viability Kernel !\n");
    }
    break;
  }

  VKObj.L_unit = L_unit;
  VKObj.Ldot_unit = Ldot_unit;
  VKObj.Theta_unit = Theta_unit;
  VKObj.Thetadot_unit = Thetadot_unit;

  std::vector<double> L_vector(L_Grids), Ldot_vector(Ldot_Grids), Theta_vector(Theta_Grids), Thetadot_vector(Thetadot_Grids);

  // L_vector filling
  for (int i = 0; i < L_Grids; i++)
  {
    L_vector[i] = LLow + (1.0 * i) * L_unit;
  }
  // Ldot_vector filling
  for (int i = 0; i < Ldot_Grids; i++)
  {
    Ldot_vector[i] = LdotLow + (1.0 * i) * Ldot_unit;
  }
  // Theta_vector filling
  for (int i = 0; i < Theta_Grids; i++)
  {
    Theta_vector[i] = ThetaLow + (1.0 * i) * Theta_unit;
  }
  // Thetadot_vector filling
  for (int i = 0; i < Thetadot_Grids; i++)
  {
    Thetadot_vector[i] = ThetadotLow + (1.0 * i) * Thetadot_unit;
  }

  // Next is to read-in all the PVKFailureMetric files.

  VKObj.ObjVec.reserve(VKObj.AngleNum);

  for (int AngInd = 0; AngInd < VKObj.AngleNum; AngInd++)
  {
    // For each PVK failure metric, we have to take their values into a vector.
    FILE * FailureMetricFile = NULL;
    int AngIndex = floor(AngInd * AngleDiff);
    string FailureMetricFileName = FailureMetricPath + "PVKFailureMetric" + std::to_string(AngIndex) + ".bin";
    const char * FailureMetricFileName_Char = FailureMetricFileName.c_str();
    FailureMetricFile = fopen(FailureMetricFileName_Char, "rb");
    std::vector<float> NodeCostVector(L_Grids * Ldot_Grids * Theta_Grids * Thetadot_Grids);
    fread(&NodeCostVector[0], sizeof(float), NodeCostVector.size(), FailureMetricFile);
    fclose(FailureMetricFile);

    // For each PVK next index, we have to take their values into a vector.
    FILE * NextIndexFile = NULL;
    string NextIndexFileName = FailureMetricPath + "PVKNextInd" + std::to_string(AngIndex) + ".bin";
    const char * NextIndexFileName_Char = NextIndexFileName.c_str();
    NextIndexFile = fopen(NextIndexFileName_Char, "rb");
    std::vector<int> NextIndexVector(L_Grids * Ldot_Grids * Theta_Grids * Thetadot_Grids);
    fread(&NextIndexVector[0], sizeof(int), NextIndexVector.size(), NextIndexFile);
    fclose(NextIndexFile);

    // Now give the value from NodeCostVector into the  ObjVec
    Eigen::Tensor<float, 4> FailureMetric_i(L_Grids, Ldot_Grids, Theta_Grids, Thetadot_Grids);
    Eigen::Tensor<int, 4>   NextIndex_i(L_Grids, Ldot_Grids, Theta_Grids, Thetadot_Grids);
    int NodeIndex = 0;
    for (int i = 0; i < L_Grids; i++)
    {
      for (int j = 0; j < Ldot_Grids; j++)
      {
        for (int k = 0; k < Theta_Grids; k++)
        {
          for (int l = 0; l < Thetadot_Grids; l++)
          {
            FailureMetric_i(i,j,k,l) = NodeCostVector[NodeIndex];
            NextIndex_i(i,j,k,l) = NextIndexVector[NodeIndex];
            NodeIndex = NodeIndex + 1;
          }
        }
      }
    }
    VKObj.ObjVec.push_back(FailureMetric_i);
    VKObj.NextIndexVec.push_back(NextIndex_i);
  }

  VKObj.L_vector = L_vector;
  VKObj.Ldot_vector = Ldot_vector;
  VKObj.Theta_vector = Theta_vector;
  VKObj.Thetadot_vector = Thetadot_vector;

  return VKObj;
}
