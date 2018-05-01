#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "MyWindow.hpp"


dart::dynamics::SkeletonPtr createKrang() {
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr krang =
      loader.parseSkeleton("/home/krang/dart/09-URDF/KrangFixedWheels/krang_fixed_wheel.urdf");
  krang->setName("krang");

  // Position its base in a reasonable way
  // Eigen::Isometry3d tf  = Eigen::Isometry3d::Identity();
  // Eigen::Isometry3d tfRot  = Eigen::Isometry3d::Identity();

  // tf.translation()      = Eigen::Vector3d(0.0, 0.0, 0.0);
  // tf *= Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX());
  // krang->getJoint(0)->setTransformFromParentBodyNode(tf);

  // tfRot = Eigen::AngleAxisd(-0.50, Eigen::Vector3d::UnitX());
  // krang->getJoint(2)->setTransformFromParentBodyNode(tfRot);

  // Get it into a useful configuration
  // krang->getDof(4)->setPosition(50.0 * M_PI / 180.0);
  // krang->getDof(5)->setPosition(-50.0 * M_PI / 180.0);

  // krang->getDof(11)->setPosition(50.0 * M_PI / 180.0);
  // krang->getDof(12)->setPosition(-20.0 * M_PI / 180.0);
  double qBaseInit = M_PI/3;
  double qWaistInit = -4*M_PI/3;
  double qTorsoInit = 0;
  double qKinectInit = 0;
  Eigen::Matrix<double, 7, 1> qLeftArmInit; 
  qLeftArmInit << 1.102, -0.589, 0.000, -1.339, 0.000, 0.3, 0.000;
  Eigen::Matrix<double, 7, 1> qRightArmInit;
  qRightArmInit << -1.102, 0.589, 0.000, 1.339, 0.000, 1.4, 0.000; 
 

  Eigen::Matrix<double, 18, 1> q;
  q << qBaseInit, qWaistInit, qTorsoInit, qKinectInit, qLeftArmInit, qRightArmInit;
  krang->setPositions(q);

  return krang;
}

int main(int argc, char* argv[])
{
  // create and initialize the world
  dart::simulation::WorldPtr world(new dart::simulation::World);
  assert(world != nullptr);

  // load skeletons
  dart::utils::DartLoader dl;
  dart::dynamics::SkeletonPtr ground  = dl.parseSkeleton("dart://sample/urdf/KR5/ground.urdf");
  dart::dynamics::SkeletonPtr robot = createKrang();

  world->addSkeleton(ground); //add ground and robot to the world pointer
  world->addSkeleton(robot);

  // create and initialize the world
  Eigen::Vector3d gravity(0.0,  -9.81, 0.0);
  world->setGravity(gravity);
  world->setTimeStep(1.0/1000);

  // create a window and link it to the world
  MyWindow window(new Controller(robot, robot->getBodyNode("lGripper"), robot->getBodyNode("rGripper") ) );
  window.setWorld(world);

  glutInit(&argc, argv);
  window.initWindow(960, 720, "Forward Simulation");
  glutMainLoop();

  return 0;
}
