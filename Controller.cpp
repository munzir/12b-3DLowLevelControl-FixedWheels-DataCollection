#include "Controller.hpp"
#include <nlopt.hpp>
#include <string>
#include <iostream>
#include <dart/dart.hpp>

using namespace dart;

#define DOF 18

//==========================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot,
                        dart::dynamics::SkeletonPtr _robotopt,
                       dart::dynamics::BodyNode* _LeftendEffector,
                       dart::dynamics::BodyNode* _RightendEffector)
  : mRobot(_robot),
    mLeftEndEffector(_LeftendEffector),
    mRightEndEffector(_RightendEffector),
    mRobotOpt(_robotopt)
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

  mSteps = 0;

    // Dump data
  dataQ.open      ("./data5/dataQ.txt");
  dataQ       << "dataQ" << endl;

  dataQref.open   ("./data5/dataQref.txt");
  dataQref    << "dataQref" << endl;

  dataQdot.open   ("./data5/dataQdot.txt");
  dataQdot    << "dataQdot" << endl;

  dataQdotdot.open("./data5/dataQdotdot.txt");
  dataQdotdot << "dataQdotdot" << endl;

  dataTorque.open ("./data5/dataTorque.txt");
  dataTorque  << "dataTorque" << endl;

  dataTime.open   ("./data5/dataTime.txt");
  dataTime    << "dataTime" << endl;

  dataM.open      ("./data5/dataM.txt");
  dataM       << "dataM" << endl;

  dataCg.open     ("./data5/dataCg.txt");
  dataCg      << "dataCg" << endl;

  dataError.open  ("./data5/dataError.txt");
  dataError   << "dataError" << endl;
}

//=========================================================================
Controller::~Controller() {}
//=========================================================================
struct OptParams {
  Eigen::MatrixXd P;
  Eigen::VectorXd b;
};

struct comOptParams {
  dynamics::SkeletonPtr robot;
  Eigen::Matrix<double, 18, 1> qRefInit;
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
//========================================================================



//========================================================================
double comOptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(my_func_data);
  Eigen::Matrix<double, 18, 1> q(x.data());
  optParams->robot->setPositions(q);

  if (!grad.empty()) {
    Eigen::Matrix<double, 1, 18> mGrad = 2*optParams->robot->getCOM()(2)*optParams->robot->getCOMLinearJacobian().block<1, 18>(2,0); 
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
  }
  
  
  return (pow(optParams->robot->getCOM()(2), 2)); 

}
//==============================================================================


//========================================================================
void constraintFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data) {
  OptParams* constParams = reinterpret_cast<OptParams *>(f_data);
  //std::cout << "done reading optParams " << std::endl;
  
  if (grad != NULL) {
    for(int i=0; i<m; i++) {
      for(int j=0; j<n; j++){
        grad[i*n+j] = constParams->P(i, j);
      }
    }
  }
  // std::cout << "done with gradient" << std::endl;

  Eigen::Matrix<double, 18, 1> X;
  for(size_t i=0; i<n; i++) X(i) = x[i];
  //std::cout << "done reading x" << std::endl;
  
  Eigen::VectorXd mResult;
  mResult = constParams->P*X - constParams->b;
  for(size_t i=0; i<m; i++) {
    result[i] = mResult(i);
  }
  // std::cout << "done calculating the result"
}
//==============================================================================


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
  
  mSteps++;

  const int dof = (const int)mRobot->getNumDofs(); // n x 1
  double dt = 0.001;

  // Define coefficients for sinusoidal, pulsation frequency for q and dq
  double wf = 0.558048373585;

  // base, waist, torso, kinect, left[1-7], right [1-7]
  Eigen::Matrix<double, 18, 4> a, b;
  a <<  0.004, 0.639, -0.011, -0.023,
        0.095, 0.502, 0.13, -0.704,
        0.181, 0.52, 0.12, -0.19,
        -0.23, 0.11, 0.076, 0.32,

        -0.009, -0.36, 0.311, -0.362,
        0.095, -0.132, -0.363, 0.474,
        -0.418, -0.25, -0.12, 0.119,
        0.023, 0.113, 0.497, 0.213,
        -0.23, -0.237, 0.153, -0.147,
        0.366, 0.366, 0.302, -0.373,
        -0.247, -0.166, 0.315, 0.031,
        
        0.206, 0.322, 0.386, 0.386,
        0.092, 0.186, 0.386, 0.386,
        0.386, 0.386, 0.386, -0.043,
        0.296, 0.262, 0.062, -0.043,
        0.386, 0.262, 0.062, 0.062,
        0.368, 0.186, 0.262, 0.186,
        0.262, 0.386, 0.28, 0.28;



  b <<  -0.051, 0.027, 0.003, 0.302,
        0.092, -0.28, 0.56, -0.136,
        -0.25, 0.009, 0.397, -0.245,
        -0.84, -0.056, -0.306, -0.53,

        -0.051, 0.027, 0.003, -0.332,
        -0.292, 0.358, -0.056, -0.436,
        -0.355, 0.039, -0.397, -0.445,
        0.328, 0.256, -0.36, 0.143,
        0.428, 0.093, 0.035, -0.28,
        -0.39, -0.085, 0.388, 0.46,
        -0.046, 0.135, -0.428, 0.387,

        -0.031, 0.386, 0.386, 0.386,
        0.298, 0.386, 0.386, 0.386,
        0.386, 0.154, 0.08, 0.08,
        0.231, 0.372, 0.367, 0.162,
        0.386, -0.043, -0.043, 0.262,
        0.319, -0.031, 0.262, -0.043,
        0.386, -0.043, 0.22, 0.18;
  

            // -0.004

  Eigen::Matrix<double, 18, 1> q0;
  q0 << 0.235, M_PI/3, -0.071, 0.095, 
        0.235, -0.004, -0.071, 0.095, -0.141, 0.208, -0.182, 
        -0.187, 0.085, -0.09, -0.014, -0.053, 0.187, 0.066;  

  // Compute joint angles & velocities using Pulsed Trajectories
  Eigen::Matrix<double, 18, 1> qref;
  Eigen::Matrix<double, 18, 1> dqref;
  qref << 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0;
  dqref = qref;

  cout << "Time: " << mTime << " and qref_vec[0]: "<< qref_vec[0] << endl;
  for (int joint = 1; joint < dof; joint++) {
    for (int l = 1; l <= 4; l++) {

      qref(joint) = qref(joint) + (a(joint, l-1)/(wf*l))*sin(wf*l*mTime)
      - (b(joint, l-1)/(wf*l))*cos(wf*l*mTime);

      dqref(joint) = dqref(joint) + a(joint,l-1)*cos(wf*l*mTime)
      + b(joint, l-1)*sin(wf*l*mTime);
      
    }
  }

  qref = qref + q0;
  qref(0) = qref_vec[0];

  comOptParams comParams;
  comParams.robot = mRobotOpt;
  comParams.qRefInit << qref; 

  OptParams constraintOptParams;
  Eigen::Matrix<double, 18, 18> P_q0 = Eigen::Matrix<double, 18, 18>::Identity();
  Eigen::Matrix<double, 18, 1>  b_q0;

  P_q0(0,0) = 0.0;
  b_q0 << qref;
  b_q0(0) = 0.0;

  constraintOptParams.P = P_q0;
  constraintOptParams.b = b_q0;

  nlopt::opt opt(nlopt::LN_COBYLA, dof);
  Eigen::VectorXd::Map(&qref_vec[0], qref.size()) = qref;
  double minf;

  double prev_qref0 = qref_vec[0];
  // for (std::vector<double>::const_iterator i = qref_vec.begin(); i != qref_vec.end(); ++i)
  //   std::cout << *i << ' ';

  opt.set_min_objective(comOptFunc, &comParams);
  std::vector<double> constraintTol(18, 1e-8);
  opt.add_equality_mconstraint(constraintFunc, &constraintOptParams, constraintTol);

  opt.set_xtol_rel(1e-4);

  //opt.set_maxtime(10);

  opt.optimize(qref_vec, minf);
  if(mSteps == 1){
    Eigen::Matrix<double, 18, 1> initialPose(qref_vec.data());
    mRobot->setPositions(initialPose);
  }

  mTime += dt;
  if (qref_vec[0] < 0.2)
  {
    qref_vec[0] = 0.2;
  }
  else if (qref_vec[0] > 0.75)
  {
    qref_vec[0] = 0.75;
  }
  
  qref(0) = qref_vec[0];
  

  dqref(0) = (mSteps==1? 0:((qref(0) - prev_qref0)/dt));
  cout << "dqref[0]: " <<dqref[0] << endl;
  
  // Get the stuff that we need
  Eigen::MatrixXd  M    = mRobot->getMassMatrix();                // n x n
  Eigen::VectorXd Cg    = mRobot->getCoriolisAndGravityForces();  // n x 1
  Eigen::VectorXd  q    = mRobot->getPositions();                 // n x 1
  Eigen::VectorXd dq    = mRobot->getVelocities();                // n x 1
  Eigen::VectorXd ddq   = mRobot->getAccelerations();             // n x 1
  Eigen::VectorXd ddqref = -mKp*(q - qref) - mKv*(dq - dqref);    // n x 1

  // Optimizer stuff
  nlopt::opt opt_qp(nlopt::LD_SLSQP, 18);
  OptParams optParams;
  std::vector<double> ddqref_vec(18);
  // cout << "Initialized optimizer variables ... " << endl << endl;

  // Perform optimization to find joint accelerations
  Eigen::Matrix<double, 18, 18> I18 = Eigen::Matrix<double, 18, 18>::Identity();

  // cout << "Passing optimizing parameters ... ";
  optParams.P = I18;
  optParams.b = ddqref;
  // cout << "Success !" << endl << endl;

  opt_qp.set_min_objective(optFunc, &optParams);
  opt_qp.set_xtol_rel(1e-4);
  //opt_qp.set_maxtime(0.005);
  opt_qp.optimize(ddqref_vec, minf);
  Eigen::Matrix<double, 18, 1> ddqRef(ddqref_vec.data());
 

 //torques
  mForces = M*ddqRef + Cg;
  Eigen::Matrix<double, 18, 18> errCoeff = Eigen::Matrix<double, 18, 18>::Identity();
  errCoeff(0,0) = 	1.0;
  errCoeff(1,1) = 	1.0;
  errCoeff(2,2) = 	1.0;
  errCoeff(3,3) = 	1.0;
  errCoeff(4,4) =  	1.0;

  errCoeff(5,5) = 	1.0;
  errCoeff(6,6) =  	1.0;
  errCoeff(7,7) = 	1.0;
  errCoeff(8,8) = 	1.3;
  errCoeff(9,9) = 	1.0;
  errCoeff(10,10) = 1.0;

  errCoeff(11,11) = 1.0;
  errCoeff(12,12) = 1.0;
  errCoeff(13,13) = 1.0;
  errCoeff(14,14) = 1.0;
  errCoeff(15,15) = 1.5;
  errCoeff(16,16) = 1.0;
  errCoeff(17,17) = 1.0;


  Eigen::VectorXd mForceErr = mForces + errCoeff*error(dq, dof);
  // Apply the joint space forces to the robot
  mRobot->setForces(mForceErr);

  //   1. State q
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
  double T = 3.1416;
  if (mTime >= 1*T ) {
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
