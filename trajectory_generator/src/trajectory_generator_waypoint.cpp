#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment

  int pieceNum = Time.size();
  MatrixXd PolyCoeff(pieceNum, 3 * p_num1d);

  return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

int TrajectoryGeneratorWaypoint::fatorial(const int n) {
    
    int result = 1;
    for (int i = 1; i <= n; i++)
    {
        result *= i;
    }
    return result;
}
 
Eigen::MatrixXd TrajectoryGeneratorWaypoint::computeEi(const double time) {
    Eigen::MatrixXd Ei = Eigen::MatrixXd::Zero(6,6);
    for(int i = 0; i < 6; ++i){
        for(int j = 0; j < 6; ++j){
            if(i == 0){
                Ei(i ,j) = pow(time, j);
                continue;
            }else if(i == 1) {
                Ei(i, j) = pow(time, j);
                continue;
            }else if(i - 1 <= j){
                Ei(i ,j) = fatorial(j) / fatorial(j - i + 1) * pow(time, j - i + 1);
            }
        }
    }
    return Ei;
}
void TrajectoryGeneratorWaypoint::minimumJerkTrajGen (
    const int pieceNum,
    const Eigen::Vector3d &initialVel,
    const Eigen::Vector3d &initialAcc,
    const Eigen::Vector3d &terminalVel,
    const Eigen::Vector3d &terminalAcc,
    const Eigen::MatrixXd &Position,
    const Eigen::VectorXd &timeAllocationVector,
    // Outputs:
    Eigen::MatrixXd &coefficientMatrix) {
    
    double Tm = timeAllocationVector[pieceNum - 1];
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6 * pieceNum , 6 * pieceNum);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(6 * pieceNum , 3);
    Eigen::MatrixXd F0 = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd Fj = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd Em = Eigen::MatrixXd::Zero(3, 6);
    Eigen::Vector3d initialPos = Position.row(0);
    Eigen::Vector3d terminalPos = Position.row(pieceNum);
    Eigen::MatrixXd intermediatePositions = Position.middleRows(1, pieceNum - 1);
    std::cout << "Position Matrix :" <<  std::endl;
    std::cout << Position << std::endl;
    std::cout << "initial Pos :" << initialPos << std::endl;
    std::cout << "terminal Pos" << terminalPos << std::endl;
    std::cout << "interPos" << intermediatePositions << std::endl;
    for(int i = 0; i < 6; ++i){
        if(i == 0) continue;
        for(int j = 0; j < 6; ++j){
            if( j == i - 1){
                Fj(i, j) = -1.0 * fatorial(j );
            }
        }
    }

    F0 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 2.0, 0.0, 0.0, 0.0;

    for(int i = 0; i < 3; ++i){
        for(int j = 0; j < 6; ++j){
            if(i <= j){
                Em(i,j) = fatorial(j) / fatorial(j - i) * pow(Tm, j - i);
            }
        }
    }
//    std::cout << "Tm: " << Tm << std::endl;
//    std::cout << "Em: " << std::endl << Em << std::endl;

    for(int i = 0; i <= pieceNum; ++i){
        if(i == 0){
            Eigen::MatrixXd D0 = Eigen::MatrixXd::Zero(3, 3);
            D0.row(0) = initialPos;
            D0.row(1) = initialVel;
            D0.row(2) = initialAcc;
            b.block(0, 0, 3, 3) = D0;
            M.block(0, 0, 3, 6) = F0;
            continue;
        }else if (i == pieceNum){
            Eigen::MatrixXd Dn = Eigen::MatrixXd::Zero(3, 3);
            Dn.row(0) = terminalPos;
            Dn.row(1) = terminalVel;
            Dn.row(2) = terminalAcc;
            b.block(6 * i - 3, 0, 3, 3) = Dn;
            M.block(6 * i - 3, 6 * i - 6, 3, 6) = Em;
            continue;
        }else{
            Eigen::MatrixXd Di = Eigen::MatrixXd::Zero(6, 3);
//            std::cout << "waypoints: " << std::endl << intermediatePositions.transpose().row(i - 1) << std::endl;
            Di.row(0) = intermediatePositions.row(i - 1);
            b.block(6 * i - 3, 0, 6, 3) = Di;
        }
        for(int j = 0; j < pieceNum; ++j){
            if(i == j){
                M.block(6 * i - 3, 6 * i, 6, 6) = Fj;
            }else if(j == i - 1){
                Eigen::MatrixXd Ei = computeEi(timeAllocationVector(i - 1));
                M.block(6 * i - 3, 6 * j, 6, 6) = Ei;
            }
        }
    }
    ROS_WARN("BIVP Solver activated");
        // std::cout << "----------------M Matrix---------------" << std::endl;
        // std::cout << M << std::endl;
        // std::cout << "----------------M Matrix---------------" << std::endl;
        // std::cout << "----------------b Matrix---------------" << std::endl;
        // std::cout << b << std::endl;
        // std::cout << "----------------b Matrix---------------" << std::endl;
        Eigen::MatrixXd M_inv = M.inverse();
        coefficientMatrix = M_inv * b;
    }