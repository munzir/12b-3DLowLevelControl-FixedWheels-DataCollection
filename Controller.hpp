#ifndef EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
#define EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_

#include <Eigen/Eigen>
#include <string>
#include <dart/dart.hpp>
#include <fstream>
#include <iostream>
using namespace std;
#define DOF 18

/// \brief Operational space controller for 6-dof manipulator
class Controller {
public:
  /// \brief Constructor
  Controller( dart::dynamics::SkeletonPtr _robot,
              dart::dynamics::SkeletonPtr _robotopt,
              dart::dynamics::BodyNode* _LeftendEffector,
              dart::dynamics::BodyNode* _RightendEffector);

  /// \brief Destructor
  virtual ~Controller();

  /// \brief
  void update(const Eigen::Vector3d& _targetPosition);

  /// \brief Get robot
  dart::dynamics::SkeletonPtr getRobot() const;

  /// \brief Get end effector of the robot
  dart::dynamics::BodyNode* getEndEffector(const std::string &s) const;

  /// \brief Keyboard control
  virtual void keyboard(unsigned char _key, int _x, int _y);

private:
  /// File handles for dumping data
  ofstream dataQ;
  ofstream dataQref;
  ofstream dataQdot;
  ofstream dataQdotdot;
  ofstream dataTorque;
  ofstream dataTime;
  ofstream dataM;
  ofstream dataCg;
  ofstream dataError;

  double mTime;
  
  /// \brief Robot
  dart::dynamics::SkeletonPtr mRobot;

  /// \brief Robot
  dart::dynamics::SkeletonPtr mRobotOpt;

  /// \brief Left End-effector of the robot
  dart::dynamics::BodyNode* mLeftEndEffector;

  /// \brief Right End-effector of the robot
  dart::dynamics::BodyNode* mRightEndEffector;

  /// \brief Control forces
  Eigen::VectorXd mForces;

  /// \brief Proportional gain for the virtual spring forces at the end effector
  Eigen::Matrix<double,18,18> mKp;

  /// \brief Derivative gain for the virtual spring forces at the end effector
  Eigen::Matrix<double,18,18> mKv;

  Eigen::Matrix<double, 18, 1> q0;

  Eigen::Matrix<double, 18, 1> q_ref_t;
  Eigen::Matrix<double, 18, 1> prev_qref;

  std::vector<double> qref_vec{-M_PI/3, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  size_t mSteps;
};

#endif  // EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_