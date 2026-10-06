//
// Created by bruce on 5/19/21.
//

#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>

namespace ocs2 {
namespace multi_robot {

using namespace std;

class Munkres
{
public:
  Munkres();
  ~Munkres();
//  double Solve(vector <vector<double> >& DistMatrix, vector<int>& Assignment);
  double Solve(Eigen::Matrix3d& DistMatrix, vector<int>& Assignment);

  // eigenshuffle helper functions
  void ndgrid(const Eigen::Vector3d& vecX, const Eigen::Vector3d& vecY, Eigen::Matrix3d& gridX, Eigen::Matrix3d& gridY);
  void meshgrid(const Eigen::Vector3d& vecX, const Eigen::Vector3d& vecY, Eigen::Matrix3d& meshX, Eigen::Matrix3d& meshY);
  Eigen::Matrix3d distancematrix(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2);

private:
  void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns);
  void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
  void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
  void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
  void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
  void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
  void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
  void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
};

} // namespace multi_robot
} // namespace ocs2