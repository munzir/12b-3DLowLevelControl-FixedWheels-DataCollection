/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Controller.hpp"
#include <nlopt.hpp>
#include <string>
#include <iostream>

//==========================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot,
                       dart::dynamics::BodyNode* _LeftendEffector,
                       dart::dynamics::BodyNode* _RightendEffector)
  : mRobot(_robot),
    mLeftEndEffector(_LeftendEffector),
    mRightEndEffector(_RightendEffector)
   {
  assert(_robot != nullptr);
  assert(_LeftendEffector != nullptr);
  assert(_RightendEffector != nullptr);

  int dof = mRobot->getNumDofs();
  std::cout << "[controller] DoF: " << dof << std::endl;

  mTime = 0;

  mForces.setZero(dof);
  mKp = Eigen::Matrix<double, 18, 18>::Zero();
  mKv = Eigen::Matrix<double, 18, 18>::Zero();

  for (int i = 0; i < dof; ++i) {
    mKp(i, i) = 750.0;
    mKv(i, i) = 250.0;
  }

  // Remove position limits
  for (int i = 0; i < dof; ++i)
    _robot->getJoint(i)->setPositionLimitEnforced(false);

  // Set joint damping
  for (int i = 0; i < dof; ++i)
    _robot->getJoint(i)->setDampingCoefficient(0, 0.5);

    // Dump data
  dataQ.open      ("../data/dataQ.txt");
  dataQ       << "dataQ" << endl;

  dataQref.open   ("../data/dataQref.txt");
  dataQref    << "dataQref" << endl;

  dataQdot.open   ("../data/dataQdot.txt");
  dataQdot    << "dataQdot" << endl;

  dataQdotdot.open("../data/dataQdotdot.txt");
  dataQdotdot << "dataQdotdot" << endl;

  dataTorque.open ("../data/dataTorque.txt");
  dataTorque  << "dataTorque" << endl;

  dataTime.open   ("../data/dataTime.txt");
  dataTime    << "dataTime" << endl;

  dataM.open      ("../data/dataM.txt");
  dataM       << "dataM" << endl;

  dataCg.open     ("../data/dataCg.txt");
  dataCg      << "dataCg" << endl;

  dataError.open  ("../data/dataError.txt");
  dataError   << "dataError" << endl;
}

//=========================================================================
Controller::~Controller() {}
//=========================================================================
struct OptParams {
  Eigen::MatrixXd P;
  Eigen::VectorXd b;
};

//=========================================================================
void printMatrix(Eigen::MatrixXd A){
  for(int i=0; i<A.rows(); i++){
    for(int j=0; j<A.cols(); j++){
      std::cout << A(i,j) << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

//========================================================================
double optFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
  OptParams* optParams = reinterpret_cast<OptParams *>(my_func_data);
  Eigen::Matrix<double, 18, 1> X(x.data());

    if (!grad.empty()) {
      Eigen::Matrix<double, 18, 1> mGrad = optParams->P.transpose()*(optParams->P*X - optParams->b);
      Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
    }
  return (0.5 * pow((optParams->P*X - optParams->b).norm(), 2));
  }

  //==============================================================================
  template <typename T> int sgn(T val) {
      return (T(0) < val) - (val < T(0));
  }

  //==============================================================================
  Eigen::MatrixXd error(Eigen::VectorXd dq, const int dof) {
  Eigen::Matrix<double, 18, 1> alpha;
  for (int i = 0; i < dof; i++) {
    double sign = sgn(dq(i));
    alpha(i) =  sign * 1/( 1 + exp(-dq(i)) ) ;
  }
  return alpha;  
}

//=========================================================================
void Controller::update(const Eigen::Vector3d& _targetPosition) {
  // using namespace dart;
  // using namespace std;

  const int dof = (const int)mRobot->getNumDofs(); // n x 1
  double dt = 0.001;

  // Define coefficients for sinusoidal, pulsation frequency for q and dq
  double wf = 0.558048373585;
  // double wf = 0.95;
  Eigen::Matrix<double, 18, 4> a, b;
  a << -0.009, -0.36, 0.311, -0.362,
        0.095, -0.132, -0.363, 0.474,
        -0.418, -0.25, -0.12, 0.119,
        0.023, 0.113, 0.497, 0.213,
        -0.23, -0.237, 0.153, -0.147,
        0.366, 0.366, 0.302, -0.373,
        -0.247, -0.166, 0.315, 0.031,
        -0.009, -0.36, 0.311, -0.362,
        0.095, -0.132, -0.363, 0.474,
        -0.418, -0.25, -0.12, 0.119,
        0.023, 0.113, 0.497, 0.213,
        -0.23, -0.237, 0.153, -0.147,
        0.366, 0.366, 0.302, -0.373,
        -0.247, -0.166, 0.315, 0.031,
        0.366, 0.366, 0.302, -0.373,
        -0.247, -0.166, 0.315, 0.031,
        -0.009, -0.36, 0.311, -0.362,
        0.095, -0.132, -0.363, 0.474;

  b <<  -0.051, 0.027, 0.003, -0.332,
        -0.292, 0.358, -0.056, -0.436,
        -0.355, 0.039, -0.397, -0.445,
        0.328, 0.256, -0.36, 0.143,
        0.428, 0.093, 0.035, -0.28,
        -0.39, -0.085, 0.388, 0.46,
        -0.046, 0.135, -0.428, 0.387,
        -0.051, 0.027, 0.003, -0.332,
        -0.292, 0.358, -0.056, -0.436,
        -0.355, 0.039, -0.397, -0.445,
        0.328, 0.256, -0.36, 0.143,
        0.428, 0.093, 0.035, -0.28,
        -0.39, -0.085, 0.388, 0.46,
        -0.046, 0.135, -0.428, 0.387,
        -0.39, -0.085, 0.388, 0.46,
        -0.046, 0.135, -0.428, 0.387,
        -0.046, 0.135, -0.428, 0.387,
        -0.051, 0.027, 0.003, -0.332;
        
  Eigen::Matrix<double, 18, 1> q0;
  q0 << 0.235, -0.004, -0.071, 0.095, -0.141, 0.208, -0.182, 0.095, 0.096,
        0.071, -0.004, -0.071, 0.095, -0.141, 0.208, -0.182, 0.095, 0.096;
  

  // Compute joint angles & velocities using Pulsed Trajectories
  Eigen::Matrix<double, 18, 1> qref;
  Eigen::Matrix<double, 18, 1> dqref;
  qref << 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0;
  dqref = qref;

  cout << "Time: " << mTime << endl;
  for (int joint = 0; joint < dof; joint++) {
    for (int l = 1; l <= 4; l++) {

      qref(joint) = qref(joint) + (a(joint, l-1)/(wf*l))*sin(wf*l*mTime)
      - (b(joint, l-1)/(wf*l))*cos(wf*l*mTime);

      dqref(joint) = dqref(joint) + a(joint,l-1)*cos(wf*l*mTime)
      + b(joint, l-1)*sin(wf*l*mTime);
      
    }
  }

  qref = qref + q0;

  mTime += dt;

  // Get the stuff that we need
  Eigen::MatrixXd  M    = mRobot->getMassMatrix();                // n x n
  Eigen::VectorXd Cg    = mRobot->getCoriolisAndGravityForces();  // n x 1
  Eigen::VectorXd  q    = mRobot->getPositions();                 // n x 1
  Eigen::VectorXd dq    = mRobot->getVelocities();                // n x 1
  Eigen::VectorXd ddq   = mRobot->getAccelerations();             // n x 1
  Eigen::VectorXd ddqref = -mKp*(q - qref) - mKv*(dq - dqref);    // n x 1

  // Optimizer stuff
  nlopt::opt opt(nlopt::LD_MMA, 18);
  OptParams optParams;
  std::vector<double> ddqref_vec(18);
  double minf;
  // cout << "Initialized optimizer variables ... " << endl << endl;

  // Perform optimization to find joint accelerations
  Eigen::Matrix<double, 18, 18> I18 = Eigen::Matrix<double, 18, 18>::Identity();

  // cout << "Passing optimizing parameters ... ";
  optParams.P = I18;
  optParams.b = ddqref;
  // cout << "Success !" << endl << endl;

  opt.set_min_objective(optFunc, &optParams);
  opt.set_xtol_rel(1e-4);
  opt.set_maxtime(0.005);
  opt.optimize(ddqref_vec, minf);
  Eigen::Matrix<double, 18, 1> ddqRef(ddqref_vec.data());
 

 //torques
  mForces = M*ddqRef + Cg;
  Eigen::Matrix<double, 18, 18> errCoeff = Eigen::Matrix<double, 18, 18>::Identity();
  errCoeff(0,0) = 	200.0;
  errCoeff(1,1) = 	500.0;
  errCoeff(2,2) = 	500.0;
  errCoeff(3,3) = 	30.0;
  errCoeff(4,4) =  	1.0;

  errCoeff(5,5) = 	30.0;
  errCoeff(6,6) =  	5.0;
  errCoeff(7,7) = 	15.0;
  errCoeff(8,8) = 	0.3;
  errCoeff(9,9) = 	25.0;
  errCoeff(10,10) = 5.0;

  errCoeff(11,11) = 15.0;
  errCoeff(12,12) = 75.0;
  errCoeff(13,13) = 4.0;
  errCoeff(14,14) = 15.0;
  errCoeff(15,15) = 0.5;
  errCoeff(16,16) = 10.0;
  errCoeff(17,17) = 15.0;


  Eigen::VectorXd mForceErr = mForces + errCoeff*error(dq, dof);
  // Apply the joint space forces to the robot
  mRobot->setForces(mForceErr);

  //    1. State q
  dataQ       << q.transpose() << endl;
  dataQref    << qref.transpose() << endl;
  dataQdot    << dq.transpose() << endl;
  dataQdotdot << ddq.transpose() << endl;
  dataTorque  << mForces.transpose() << endl;
  dataTime    << mTime << endl;
  dataM       << M << endl;
  dataCg      << Cg.transpose() << endl;
  dataError   << (errCoeff*error(dq, dof)).transpose() << endl;

  // Closing operation
  double T = 2*3.1416/wf;
  if (mTime >= 2*T ) {
    cout << "Time period met. Stopping data recording ...";
    dataQ.close();
    dataQref.close();
    dataQdot.close();
    dataQdotdot.close();
    dataTorque.close();
    dataTime.close();
    dataM.close();
    dataCg.close();
    dataError.close();
    cout << "File handles closed!" << endl << endl << endl;
    exit (EXIT_FAILURE);
  }  
}


//=========================================================================
dart::dynamics::SkeletonPtr Controller::getRobot() const {
  return mRobot;
  //
}

//=========================================================================
dart::dynamics::BodyNode* Controller::getEndEffector(const std::string &s) const {
  if (s.compare("left")) {  return mLeftEndEffector; }
  else if (s.compare("right")) { return mRightEndEffector; }
}

//=========================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {
}
