// This function is used to conduct the Polyhedron Projection using Double Description Method

#include "RobotInfo.h"
#include "CommonHeader.h"
#include <setoper.h>
#include <cdd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <bits/stdc++.h>
#include <sstream>
#include <iterator>

static string cone_file_name = "CCV";

static double eps = 1e-8;

static void String2Coordinates(const string& str_line, std::vector<Vector3>& ConeVertices)
{
  // This function is used to transform string into coordinates
  std::istringstream buf(str_line);
  std::istream_iterator<std::string> beg(buf), end;
  std::vector<std::string> tokens(beg, end); // done!
  int Type = std::stoi (tokens[0]);
  switch (Type)
  {
    case 1:
    {
      double x = std::stod(tokens[1]);
      double y = std::stod(tokens[2]);
      Vector3 ConeVertex(x, y, 0.0);
      ConeVertices.push_back(ConeVertex);
    }
    break;
    default:
    break;
  }
}

static std::vector<Vector3> INEReader()
{
  // This function is used to read in the cone file.
  string cone_file_path = cone_file_name + ".ext";
  ifstream ConeInfofile (cone_file_path);

  string str_line;
  string start_keyword="real";
  string end_keyword="end";

  std::vector<Vector3>  ConeVertices;
  int valid_flag = 0;
  if (ConeInfofile.is_open())
  {
    while (getline (ConeInfofile, str_line))
    {
      int start_pos = str_line.find(start_keyword);
      int end_pos = str_line.find(end_keyword);
      if(end_pos != string::npos)
      {
        return ConeVertices;
      }
      switch (valid_flag)
      {
        case 1:
        {
          String2Coordinates(str_line, ConeVertices);
        }
        break;
        default:
        break;
      }
      if (start_pos != string::npos)
      {
        valid_flag = 1;
      }
    }
    ConeInfofile.close();
  }
  else cout << "Unable to open file";

  return ConeVertices;
}

static void INEWriter(const std::vector<Vector3>& ConeInequalities)
{
  // It has been noticed that based on the pure cddlib, the generated vertices are not correct.
  // As a result, a file write/read process has to be conducted.
  string cone_file_path = cone_file_name + ".ine";
  ofstream ConeInfofile (cone_file_path);
  /*
      Header
  */
  ConeInfofile<<"H-representation"<<endl;
  ConeInfofile<<"begin"<<endl;
  ConeInfofile<<" "<<std::to_string(ConeInequalities.size())<<" 3 real"<<endl;

  for (int i = 0; i < ConeInequalities.size(); i++)
  {
    Vector3 ConeInequality = ConeInequalities[i];
    ConeInfofile<<"\t"<<std::to_string(ConeInequality.x)<<" "<<std::to_string(ConeInequality.y)<<" "<<std::to_string(ConeInequality.z)<<endl;
  }
  ConeInfofile<<"end"<<endl;
  ConeInfofile.close();
}

static Matrix PolyhedralProjection(const std::vector<Vector3> & ActContactPositions, const std::vector<Vector3> & ConeUnit, const int & EdgeNo, int &FailureFlag)
{
  // This function is used to project the feasible contact wrench cone
  /*
    For each of the vector in ConeUnit, a new projected Conic Vector should be computed.
  */
  dd_PolyhedraPtr poly;
  dd_MatrixPtr A, B, G;
  dd_rowrange m;
  dd_colrange d;
  dd_ErrorType err;

  FailureFlag = 0;

  dd_set_global_constants();                  /* First, this must be called to use cddlib. */

  m = ConeUnit.size();
  d = 6 + 1;        // Here 6 means the force and momentum balance

  A = dd_CreateMatrix(m,d);
  for (int i = 0; i < ConeUnit.size(); i++)
  {
    for (int j = 0; j < d; j++)
    {
      dd_set_d(A->matrix[i][j], 0.0);
    }
    int EleIndex = floor(i/EdgeNo);
    Vector3 ContactPos = ActContactPositions[EleIndex];
    // Give the value into A matrix.
    Vector3 ConeUnit_i = ConeUnit[i] - ContactPos;
    dd_set_d(A->matrix[i][1], ConeUnit_i.x);
    dd_set_d(A->matrix[i][2], ConeUnit_i.y);
    dd_set_d(A->matrix[i][3], ConeUnit_i.z);

    Vector3 pcrossf = cross(ContactPos, ConeUnit_i);
    dd_set_d(A->matrix[i][4], pcrossf.x);
    dd_set_d(A->matrix[i][5], pcrossf.y);
    dd_set_d(A->matrix[i][6], pcrossf.z);
  }

  A->representation = dd_Generator;
  poly=dd_DDMatrix2Poly(A, &err);

  G=dd_CopyGenerators(poly);
  // dd_WriteMatrix(stdout,G);   printf("\n");
  B=dd_CopyInequalities(poly);
  // dd_WriteMatrix(stdout,B);   printf("\n");

  Matrix H;
  if (err!=dd_NoError)
  {
    // printf("PolyhedralProjection: Given Matrix is not compatiable!\n");
    FailureFlag = 1;
    dd_FreeMatrix(A);
    dd_FreeMatrix(B);
    dd_FreeMatrix(G);
    dd_FreePolyhedra(poly);

    dd_free_global_constants();  /* At the end, this must be called. */

    return H;
  }

  H.resize(B->rowsize, B->colsize-1);

  for (int i = 0; i < B->rowsize; i++)
  {
    for (int j = 0; j < B->colsize-1; j++)
    {
      H(i,j) = *B->matrix[i][j+1];
    }
  }
  dd_FreeMatrix(A);
  dd_FreeMatrix(B);
  dd_FreeMatrix(G);
  dd_FreePolyhedra(poly);

  dd_free_global_constants();  /* At the end, this must be called. */
  H.setNegative(H);
  return H;
}

static std::vector<Vector3> CentroidalCone(const Matrix& H, const Vector3& COMPos, const Vector3& COMVel)
{
  // This function is used to calcualte the centroidal Cone based on the H matrix
  Vector3 g(0.0, 0.0, -9.81);
  // According to the Zero-Step Capturability Paper there are three terms to be computed.
  Vector a_lhs(6);
  Vector3 v = COMVel;
  v.setNormalized(v);

  // For term a,
  a_lhs[0] = 1.0;
  a_lhs[1] = 1.0;
  a_lhs[2] = 1.0;
  Vector3 mczerocrossv = 1.0 * cross(COMPos, v);
  a_lhs[3] = mczerocrossv.x;
  a_lhs[4] = mczerocrossv.y;
  a_lhs[5] = mczerocrossv.z;

  // For term b,
  Vector b_lhs(6);
  Vector3 mgcrossv = 1.0 * cross(g, v);
  b_lhs[0] = 0.0;
  b_lhs[1] = 0.0;
  b_lhs[2] = 0.0;
  b_lhs[3] = mgcrossv.x;
  b_lhs[4] = mgcrossv.y;
  b_lhs[5] = mgcrossv.z;

  // For term d,
  Vector d_rhs(6);
  Vector3 mczerocrossg = 1.0 * cross(COMPos, g);
  d_rhs[0] = 0.0;
  d_rhs[1] = 0.0;
  d_rhs[2] = 1.0 * -9.81;
  d_rhs[3] = mczerocrossg.x;
  d_rhs[4] = mczerocrossg.y;
  d_rhs[5] = mczerocrossg.z;

  Vector Ha, Hb, Hd;
  H.mul(a_lhs,Ha);
  H.mul(b_lhs,Hb);
  H.mul(d_rhs,Hd);

  /*
    For each of the vector in ConeUnit, a new projected Conic Vector should be computed.
  */
  dd_PolyhedraPtr poly;
  dd_MatrixPtr A, B, G;
  dd_rowrange m;
  dd_colrange d;
  dd_ErrorType err;

  dd_set_global_constants();                  /* First, this must be called to use cddlib. */

  m = Ha.size();
  d = 2 + 1;

  A = dd_CreateMatrix(m,d);
  for (int i = 0; i < m; i++)
  {
    dd_set_d(A->matrix[i][0], Hd[i]);
    dd_set_d(A->matrix[i][1], -1.0 * Hb[i]);
    dd_set_d(A->matrix[i][2], -1.0 * Ha[i]);
  }
  A->representation = dd_Inequality;
  poly=dd_DDMatrix2Poly(A, &err);  /* compute the second (generator) representation */
  if (err!=dd_NoError)
  {
    // printf("CentroidalCone: Given Matrix is not compatiable!\n");
  }
  G=dd_CopyGenerators(poly);
  // dd_WriteMatrix(stdout,G);   printf("\n");
  B=dd_CopyInequalities(poly);
  // dd_WriteMatrix(stdout,B);   printf("\n");

  std::vector<Vector3> ConeInequalities;
  ConeInequalities.reserve(B->rowsize);
  for (int i = 0; i < B->rowsize; i++)
  {
    Vector3 ConeInequality(*B->matrix[i][0], *B->matrix[i][1], *B->matrix[i][2]);
    ConeInequalities.push_back(ConeInequality);
  }
  dd_FreeMatrix(A);
  dd_FreeMatrix(B);
  dd_FreeMatrix(G);
  dd_FreePolyhedra(poly);

  dd_free_global_constants();  /* At the end, this must be called. */

  string del_command = "rm -f " + cone_file_name + ".*";
  const char *delete_command = del_command.c_str();
  std::system(delete_command);

  INEWriter(ConeInequalities);

  string cone_command = "./../../../cddplus/cddf+ " + cone_file_name + ".ine";

  // Convert string to const char * as system requires
  // parameter of type const char *
  const char *command = cone_command.c_str();
  std::system(command);

  std::vector<Vector3> ConeVertices = INEReader();

  // string del_command = "rm -f " + cone_file_name + ".*";
  // const char *delete_command = del_command.c_str();
  // std::system(delete_command);

  return ConeVertices;
}

static std::pair<double, double> IntegratorLDS(const double & alpha, const double & alphadot, double & alphaddot, const double & beta, const double & gamma, const double & alphaupp)
{
  // This function is used to integrate the system dynamics according to the a-addot figure
  /*
  Here: alphaddot = beta + gamma * alpha,         alphalow<=alpha<=alphaupp
  */
  std::pair <double, double> ResPair;

  if (gamma>eps)
  {
    double omega = sqrt(gamma);
    double arg = -1.0 * alphadot/(omega * (alpha + beta/gamma));
    if((arg * arg)<=1.0)
    {
      double t = 1.0/omega * atanh(arg);
      double alpha_t = cosh(omega * t) * alpha + sinh(omega * t) * alphadot/omega + (cosh(omega * t) - 1.0) * (beta/gamma);
      if(alpha_t<=alphaupp)
      {
        alphaddot = beta + (alpha_t - alpha) * gamma;
        ResPair = std::make_pair(alpha_t, 0.0);
        return ResPair;
      }
    }
    double A = alpha + beta/gamma;
    double B = alphadot/omega;
    double C = -alphaupp - beta/gamma;
    double InLog = (sqrt(B * B + C * C - A * A) - C)/(A + B);
    double t = 1.0/omega * log(InLog);
    double alpha_t = cosh(omega * t) * alpha + sinh(omega*t)/omega * alphadot + (cosh(omega * t)- 1.0) * beta/gamma;
    double alphadot_t = omega * sinh(omega * t) * alpha + cosh(omega*t) * alphadot + omega * sinh(omega * t) * beta/gamma;
    alphaddot = beta + (alpha_t - alpha) * gamma;
    ResPair = std::make_pair(alphaupp, alphadot_t);
    return ResPair;
  }
  else
  {
    if (gamma<-eps)
    {
      // std::cout<<"Needs to be careful since gamma is negative!"<<std::endl;
      // system("pause");
      std::complex<double> omega(0.0, sqrt(-1.0 * gamma));                          // This is an imaginary number.
      std::complex<double> arg = -1.0 * alphadot/(omega * (alpha + beta/gamma));    // This is an imaginary number.
      std::complex<double> t = 1.0/omega * atanh(arg);                              // This is a real number.
      std::complex<double> alpha_t = cosh(omega * t) + sinh(omega * t) * alphadot/omega + (cosh(omega * t)-1.0) * (beta/gamma); // This is also a real number.
      double alpha_t_real = real(alpha_t);
      if(alpha_t_real <= alphaupp)
      {
        alphaddot = beta + (alpha_t_real - alpha) * gamma;
        ResPair = std::make_pair(alpha_t_real, 0.0);
        return ResPair;
      }
      std::complex<double> A = alpha + beta/gamma;
      std::complex<double> B = alphadot/omega;
      std::complex<double> C = -alphaupp - beta/gamma;
      std::complex<double> InLog = (-sqrt(B * B + C * C - A * A) - C)/(A + B);
      t = 1.0/omega * log(InLog);
      std::complex<double> alphadot_t_complex = omega * sinh(omega * t) * alpha + cosh(omega*t) * alphadot + omega * sinh(omega * t) * beta/gamma;
      std::complex<double> alpha_t_complex = cosh(omega * t) * alpha + sinh(omega*t)/omega * alphadot + (cosh(omega * t)- 1.0) * beta/gamma;
      alpha_t_real = real(alpha_t_complex);
      double alphadot_t = real(alphadot_t_complex);
      alphadot_t = sqrt(alphadot_t * alphadot_t);
      alphaddot = beta + (alpha_t_real - alpha) * gamma;
      ResPair = std::make_pair(alphaupp, alphadot_t);
      return ResPair;
    }
    else
    {
      double alpha_t = alpha - 0.5 * alphadot * alphadot/beta;
      if(alpha_t<=alphaupp)
      {
        alphaddot = beta + (alpha_t - alpha) * gamma;
        ResPair = std::make_pair(alpha_t, 0.0);
        return ResPair;
      }
      double t = (-alphadot + sqrt(alphadot * alphadot - 2.0 * beta * (alpha - alphaupp)))/beta;
      alpha_t = alphaupp;
      double alphadot_t = alphadot + t * beta;
      alphaddot = beta + (alpha_t - alpha) * gamma;
      ResPair = std::make_pair(alpha_t, alphadot_t);
      return ResPair;
    }
  }
  return ResPair;
}

static double LinearInter(const Vector2& LeftPt, const Vector2& RightPt, const double& x_new)
{
  // This function is used to calculate the value for y_new based on the LeftPt and Rightpt.
  double x_offset = x_new - LeftPt.x;
  double k = (RightPt.y - LeftPt.y)/(RightPt.x - LeftPt.x);
  double y_new = LeftPt.y + k * x_offset;
  return y_new;
}

static void AccLinearCoeff(const double & Alpha, const std::vector<Vector2> & Path, double & Beta, double & Gamma, double & AlphaUpp, const double & AlphaFeasMax)
{
  // This function is used to find out the linear coefficients for acceleration function.
  for (int i = 0; i < Path.size()-1; i++)
  {
    if((Alpha>=Path[i].x)&&(Alpha<=Path[i+1].x))
    {
      // This is used to bound the range of alpha.
      Vector2 LeftPt = Path[i];
      Vector2 RightPt = Path[i+1];

      Gamma = (RightPt.y - LeftPt.y)/(RightPt.x - LeftPt.x);
      Beta = LeftPt.y - Gamma * LeftPt.x;

      if(RightPt.y<0)
      {
        AlphaUpp = RightPt.x;
      }
      else
      {
        AlphaUpp = LeftPt.x - LeftPt.y/Gamma;
      }
    }
  }
  if(AlphaUpp>=AlphaFeasMax)
  {
    AlphaUpp = AlphaFeasMax;
  }
}

static int LinearCoeff(const double & alpha, const FacetInfo & FacetObj, const int & Flag, double & beta_i, double & gamma_i, double & alphaupp_i)
{
  // This function is used to calculate the linear coefficient: alphaddot = beta_i + gamma_i * alpha.
  std::vector<int> EdgeIndices;
  std::vector<Vector2> LeftPts, RightPts;
  std::vector<double> InterAlphaddot;
  for (int i = 0; i < FacetObj.FacetEdges.size(); i++)
  {
    double alphaleft = FacetObj.FacetEdges[i].first.x - alpha;
    double alpharight = FacetObj.FacetEdges[i].second.x - alpha;
    if(((alphaleft<=0)&&(alpharight>=0))||((alphaleft >=0)&&(alpharight<=0)))
    {
      // Then we switch the order of first, second to make sure that the second point is on the right.
      Vector2 LeftPt, RightPt;
      if(FacetObj.FacetEdges[i].second.x>FacetObj.FacetEdges[i].first.x)
      {
        LeftPt.x = FacetObj.FacetEdges[i].first.x;
        LeftPt.y = FacetObj.FacetEdges[i].first.y;

        RightPt.x = FacetObj.FacetEdges[i].second.x;
        RightPt.y = FacetObj.FacetEdges[i].second.y;
      }
      else
      {
        LeftPt.x = FacetObj.FacetEdges[i].second.x;
        LeftPt.y = FacetObj.FacetEdges[i].second.y;

        RightPt.x = FacetObj.FacetEdges[i].first.x;
        RightPt.y = FacetObj.FacetEdges[i].first.y;
      }
      if((RightPt.x - alpha)>0)
      {
        // This means that we only choose the point on the right side
        EdgeIndices.push_back(i);
        LeftPts.push_back(LeftPt);
        RightPts.push_back(RightPt);
        double alphaddot = LinearInter(LeftPt, RightPt, alpha);
        InterAlphaddot.push_back(alphaddot);
      }
    }
  }
  if(InterAlphaddot.size() ==0)
  {
    // This means that the current region does not contain the origin to search for the feasible solution.
    return 2;
  }

  // In this case, we should sort alpha values according to the linear splines.
  int AlphaIndex;
  switch (Flag)
  {
    case 0:
    {
      AlphaIndex = std::max_element(InterAlphaddot.begin(), InterAlphaddot.end()) - InterAlphaddot.begin();
    }
    break;
    case 1:
    {
      AlphaIndex = std::min_element(InterAlphaddot.begin(), InterAlphaddot.end()) - InterAlphaddot.begin();
    }
    break;
    default:
    break;
  }
  Vector2 LeftPt = LeftPts[AlphaIndex];
  Vector2 RightPt = RightPts[AlphaIndex];

  gamma_i = (RightPt.y - LeftPt.y)/(RightPt.x - LeftPt.x);
  beta_i = InterAlphaddot[AlphaIndex];
  if(RightPt.y<0)
  {
    alphaupp_i = RightPt.x;
  }
  else
  {
    alphaupp_i = LeftPt.x - LeftPt.y/gamma_i;
  }
  return 1;
}

static Vector2 AppendPoint(const Vector2 & Point, const FacetInfo & FacetObj, bool & PointAdditionFlag)
{
  Vector2 NextPoint;
  for (int i = 0; i < FacetObj.FacetEdges.size(); i++)
  {
    double FirstCom_x = FacetObj.FacetEdges[i].first.x - Point.x;
    double FirstCom_y = FacetObj.FacetEdges[i].first.y - Point.y;
    double FirstComVal = FirstCom_x * FirstCom_x + FirstCom_y * FirstCom_y;

    double SecondCom_x = FacetObj.FacetEdges[i].second.x - Point.x;
    double SecondCom_y = FacetObj.FacetEdges[i].second.y - Point.y;
    double SecondComVal = SecondCom_x * SecondCom_x + SecondCom_y * SecondCom_y;

    if(FirstComVal<eps)
    {
      // This means that First Point is the Point
      if(SecondCom_x>0)
      {
        NextPoint.x = FacetObj.FacetEdges[i].second.x;
        NextPoint.y = FacetObj.FacetEdges[i].second.y;

        PointAdditionFlag = true;
        return NextPoint;
      }
    }

    if(SecondComVal<eps)
    {
      if(FirstCom_x>0)
      {
        NextPoint.x = FacetObj.FacetEdges[i].first.x;
        NextPoint.y = FacetObj.FacetEdges[i].first.y;

        PointAdditionFlag = true;
        return NextPoint;
      }
    }
  }
  PointAdditionFlag = false;
  return NextPoint;
}

static void FindPathToTheEnd(std::vector<Vector2> & Path, const FacetInfo & FacetObj)
{
  bool PointAdditionFlag = true;
  while(PointAdditionFlag)
  {
    // try to find the path connecting to the last point from path
    Vector2 NextPoint;
    NextPoint = AppendPoint(Path[Path.size()-1], FacetObj, PointAdditionFlag);
    switch (PointAdditionFlag)
    {
      case true:
      {
        Path.push_back(NextPoint);
      }
      break;
      default:
      {
      }
      break;
    }
  }
}

static bool PathToTheEnd(const FacetInfo & FacetObj, std::vector<Vector2> & UpperPath, std::vector<Vector2> & LowerPath)
{
  // This function is used to find the path from 0 to the end.
  bool PathFlag = false;
  std::vector<Vector2> StartPointsLeft, StartPointsRight;
  std::vector<int> InterSegIndices;
  for (int i = 0; i < FacetObj.FacetEdges.size(); i++)
  {
    // The first element is the alpha while the second element is the alpha ddot.
    double Sign = FacetObj.FacetEdges[i].first.x * FacetObj.FacetEdges[i].second.x;
    if (Sign<0)
    {
      double delta_y = FacetObj.FacetEdges[i].first.y - FacetObj.FacetEdges[i].second.y;
      double delta_x = FacetObj.FacetEdges[i].first.x - FacetObj.FacetEdges[i].second.x;
      double k = delta_y/delta_x;

      double StartPointLeft_y = FacetObj.FacetEdges[i].first.y - k * FacetObj.FacetEdges[i].first.x;
      Vector2 StartPointLeft(0.0, StartPointLeft_y);
      StartPointsLeft.push_back(StartPointLeft);
      if(FacetObj.FacetEdges[i].first.x>FacetObj.FacetEdges[i].second.x)
      {
        Vector2 StartPointRight(FacetObj.FacetEdges[i].first.x, FacetObj.FacetEdges[i].first.y);
        StartPointsRight.push_back(StartPointRight);
      }
      else
      {
        Vector2 StartPointRight(FacetObj.FacetEdges[i].second.x, FacetObj.FacetEdges[i].second.y);
        StartPointsRight.push_back(StartPointRight);
      }
      InterSegIndices.push_back(i);
    }
    else
    {
      if(Sign == 0)
      {
        double FirstAtZero = FacetObj.FacetEdges[i].first.x * FacetObj.FacetEdges[i].first.x;
        if(FirstAtZero<eps)
        {
          if(FacetObj.FacetEdges[i].first.x<FacetObj.FacetEdges[i].second.x)
          {
            Vector2 StartPointLeft(FacetObj.FacetEdges[i].first.x, FacetObj.FacetEdges[i].first.y);
            Vector2 StartPointRight(FacetObj.FacetEdges[i].second.x, FacetObj.FacetEdges[i].second.y);
            StartPointsLeft.push_back(StartPointLeft);
            StartPointsRight.push_back(StartPointRight);
            InterSegIndices.push_back(i);
          }
        }
        else
        {
          if(FacetObj.FacetEdges[i].first.x>FacetObj.FacetEdges[i].second.x)
          {
            Vector2 StartPointLeft(FacetObj.FacetEdges[i].second.x, FacetObj.FacetEdges[i].second.y);
            Vector2 StartPointRight(FacetObj.FacetEdges[i].first.x, FacetObj.FacetEdges[i].first.y);
            StartPointsLeft.push_back(StartPointLeft);
            StartPointsRight.push_back(StartPointRight);
            InterSegIndices.push_back(i);
          }
        }
      }
    }
  }

  switch (StartPointsLeft.size())
  {
    case 0:
    {
      return false;
    }
    break;
    case 3:
    {
      std::cerr<<"Intersections cannot have more than 2 segments!"<<endl;
      return false;
    }
    break;
    default:
    {
    }
    break;
  }

  // Now the job is to figure out which one is at top and which one is at bottom.
  if(StartPointsLeft[0].y > StartPointsLeft[1].y)
  {
    UpperPath.push_back(StartPointsLeft[0]);
    UpperPath.push_back(StartPointsRight[0]);

    LowerPath.push_back(StartPointsLeft[1]);
    LowerPath.push_back(StartPointsRight[1]);
  }
  else
  {
    UpperPath.push_back(StartPointsLeft[1]);
    UpperPath.push_back(StartPointsRight[1]);

    LowerPath.push_back(StartPointsLeft[0]);
    LowerPath.push_back(StartPointsRight[0]);
  }

  // Alright, let's find the path from this point to the most right end.
   FindPathToTheEnd(UpperPath, FacetObj);
   FindPathToTheEnd(LowerPath, FacetObj);

  return true;
}

static bool TooFastCaseValidation(const double & AlphadotInit, const std::vector<Vector2> & UppPath, const double & AlphaFeasMin, const double & AlphaFeasMax)
{
  // This main purpose of this function is to validate whether robot can reach feasible alpha region.
  // This computation will be terminated if any of these two conditions has been triggered.
  // 1. alpha reaches feasible region.
  // 2. alphadot has been declined to zero.
  double Alpha = 0.0;
  double Alphadot = AlphadotInit;
  double Beta = 0.0;
  double Gamma = 0.0;
  double AlphaUpp = 0.0;
  double Alphaddot = 0.0;

  int IterLimit = 20;
  for (int i = 0; i < IterLimit; i++)
  {
    AccLinearCoeff(Alpha, UppPath, Beta, Gamma, AlphaUpp, AlphaFeasMax);
    std::pair<double, double> AlphaNAlphadot = IntegratorLDS(Alpha, Alphadot, Alphaddot, Beta, Gamma, AlphaUpp);
    Alpha = AlphaNAlphadot.first;
    Alphadot = AlphaNAlphadot.second;

    // Condition 1
    if(Alpha>=AlphaFeasMin)
    {
      return true;
    }

    // Condition 2
    if((Alphadot==0.0)&&(Alpha<AlphaFeasMin))
    {
      return false;
    }
  }
  return false;
}

static bool ZSCEvaluation(const double & AlphadotInit, const std::vector<Vector2> & LowPath, const std::vector<Vector2> & UppPath, const double & AlphaFeasMin, const double & AlphaFeasMax, const double & AlphaMax)
{
  // This function is used to conduct the evaluation of ZSC.
  int MaxIter = 100;
  int CurIter = 0;
  double Alpha = 0.0;
  double Alphadot = AlphadotInit;
  double Alphaddot = 0.0;

  double Beta = 0.0;
  double Gamma = 0.0;
  double AlphaUpp = 0.0;          // This is the Upper Bound for a certain integration.

  bool StabilizedFlag = false;
  while (CurIter<MaxIter)
  {
    if((Alphadot>-eps)&&(Alphadot<eps))
    {
      // In this case, Alphadot vanishes!
      if((Alpha=AlphaFeasMin)&&(Alpha<=AlphaFeasMax))
      {
        // C3.1
        // Tested!
        StabilizedFlag = true;
        // std::printf("C3.1\n");
        return StabilizedFlag;
      }
      if(Alpha<AlphaFeasMin)
      {
        // C3.2
        // This is an annoying case.
        StabilizedFlag = TooFastCaseValidation(AlphadotInit, UppPath, AlphaFeasMin, AlphaFeasMax);
        // std::printf("C3.2\n");
        return StabilizedFlag;
      }
      if(Alpha>AlphaFeasMax)
      {
        // C3.3
        StabilizedFlag = false;
        // std::printf("C3.3\n");
        return StabilizedFlag;
      }
    }
    else
    {
      // In this case, robot's centroidal velocity is not zero.
      if(Alpha>=AlphaFeasMax)
      {
        // C1: Velocity is too large to be compensated!
        // Tested!
        StabilizedFlag = false;
        // std::printf("C1\n");
        return StabilizedFlag;
      }
      if(Alpha>=AlphaMax)
      {
        // C2: Position arrives at the position where constraints will be violated!
        // Tested!
        StabilizedFlag = false;
        // std::printf("C2\n");
        return StabilizedFlag;
      }
    }
    AccLinearCoeff(Alpha, LowPath, Beta, Gamma, AlphaUpp, AlphaFeasMax);
    std::pair<double, double> AlphaNAlphadot = IntegratorLDS(Alpha, Alphadot, Alphaddot, Beta, Gamma, AlphaUpp);
    Alpha = AlphaNAlphadot.first;
    Alphadot = AlphaNAlphadot.second;

    // std::printf("Alpha: %f and Alphadot: %f\n", Alpha, Alphadot);
    CurIter = CurIter + 1;
  }
  return StabilizedFlag;
}

static bool ZSCInner(const std::vector<Vector3> & ActContactPositions, const std::vector<Vector3> & ConeUnit, const int & EdgeNo, const Vector3& COMPos, const Vector3& COMVel)
{
  int HFailureFlag;
  Matrix H = PolyhedralProjection(ActContactPositions, ConeUnit, EdgeNo, HFailureFlag);
  switch (HFailureFlag)
  {
    case 1:
    {
      // This means that Zero-step capturability method does not work
      return false;
    }
    break;
    default:
    break;
  }

  std::vector<Vector3> ConeVertices = CentroidalCone(H, COMPos, COMVel);
  if(ConeVertices.size()<3)
  {
    // In this case, there is no need to conduct further computation.
    return false;
  }

  int CollinearFlag = CollinearTest(ActContactPositions);
  switch (CollinearFlag)
  {
    case 1:
    {
      return false;
    }
    break;
    default:
    break;
  }

  int FacetFlag = 0;
  FacetInfo FacetObj = FlatContactHullGeneration(ConeVertices, FacetFlag);
  switch (FacetFlag)
  {
    case 0:
    {
      return false;
    }
    break;
    default:
    break;
  }

  // Now let's compute the maximum deaccelerated velocity.
  bool StabilizedFlag = true;
  /*
    The first job is to compute the intersection where the acceleration vanishes.
  */
  std::vector<double> AlphaFeasible;
  std::vector<double> AlphaTotal(FacetObj.FacetEdges.size());
  for (int i = 0; i < FacetObj.FacetEdges.size(); i++)
  {
    // The first element is the alpha while the second element is the alpha ddot.
    double Sign = FacetObj.FacetEdges[i].first.y * FacetObj.FacetEdges[i].second.y;
    if (Sign<0)
    {
      double delta_y = FacetObj.FacetEdges[i].first.y - FacetObj.FacetEdges[i].second.y;
      double delta_x = FacetObj.FacetEdges[i].first.x - FacetObj.FacetEdges[i].second.x;
      double k = delta_y/delta_x;
      double Alpha = FacetObj.FacetEdges[i].first.x - FacetObj.FacetEdges[i].first.y/k;
      AlphaFeasible.push_back(Alpha);
    }
    else
    {
      if(Sign == 0)
      {
        double FirstAtZero = FacetObj.FacetEdges[i].first.y * FacetObj.FacetEdges[i].first.y;
        if(FirstAtZero<eps)
        {
          AlphaFeasible.push_back(FacetObj.FacetEdges[i].first.x);
        }
        else
        {
          AlphaFeasible.push_back(FacetObj.FacetEdges[i].second.x);
        }
      }
    }
    AlphaTotal[i] = FacetObj.FacetEdges[i].first.x;
  }
  switch (AlphaFeasible.size())
  {
    case 0:
    {
      return false;
    }
    break;
    default:
    break;
  }

  double AlphaFeasMin = *std::min_element(AlphaFeasible.begin(), AlphaFeasible.end());
  double AlphaFeasMax = *std::max_element(AlphaFeasible.begin(), AlphaFeasible.end());

  double AlphaTotalMin = *std::min_element(AlphaTotal.begin(), AlphaTotal.end());
  double AlphaTotalMax = *std::max_element(AlphaTotal.begin(), AlphaTotal.end());

  std::vector<Vector2> UpperPath, LowerPath;
  PathToTheEnd(FacetObj, UpperPath, LowerPath);

  switch (UpperPath.size())
  {
    case 0:
    {
      return false;
    }
    break;
    case 1:
    {
      return false;
    }
    break;
    default:
    {
    }
    break;
  }

  switch (LowerPath.size())
  {
    case 0:
    {
      return false;
    }
    break;
    case 1:
    {
      return false;
    }
    break;
    default:
    {
    }
    break;
  }

  double AlphadotInit = sqrt(COMVel.x * COMVel.x + COMVel.y * COMVel.y + COMVel.z * COMVel.z);

  StabilizedFlag = ZSCEvaluation(AlphadotInit, LowerPath, UpperPath, AlphaFeasMin, AlphaFeasMax, AlphaTotalMax);

  return StabilizedFlag;
}

double ZeroStepCapturabilityGenerator(const std::vector<Vector3> & ActContactPositions, const std::vector<Vector3> & ConeUnit, const int & EdgeNo, const Vector3& COMPos, const Vector3& COMVel)
{
  // This function is used to generate the solution for zero capturability failure metric.
  bool StabilizedFlag = ZSCInner(ActContactPositions, ConeUnit, EdgeNo, COMPos, COMVel);
  double ZSCObj = 1.0;
  switch (StabilizedFlag)
  {
    case true:
    {
      // This means that the robot can be stabilized using this method.
      ZSCObj = 0.0;
    }
    break;
    default:
    {
      // This means that the robot cannot be stabilized using this metric.
      ZSCObj = 1.0;
    }
    break;
  }
  return ZSCObj;
}
