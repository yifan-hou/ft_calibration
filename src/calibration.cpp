#include <iostream>

#include <RobotUtilities/utilities.h>

#include <hardware_interfaces/robot_interfaces.h>
#include <hardware_interfaces/ft_interfaces.h>

#include <ati_netft/ati_netft.h>
#include <abb_egm/abb_egm.h>
#include <ur_socket/ur_socket.h>


using namespace std;
using namespace RUT;

#define PI 3.1415926

// Global variables
RobotInterfaces *_robot;
FTInterfaces *_ft;
double _kVelMaxTrans, _kAccMaxTrans, _kVelMaxRot, _kAccMaxRot;
double _fHz;

void moveTo(double *pose_set) {
  double pose0[7], pose_send[7];
  _robot->getCartesian(pose0);

  MatrixXd pose_traj;
  cout << "[Motion planning]\n";
  cout << "pose0: " << pose0[0] << "|" << pose0[1] << "|" << pose0[2] << "|" << pose0[3] << "|" << pose0[4] << "|" << pose0[5] << "|" << pose0[6] << endl;
  cout << "pose_set: " << pose_set[0] << "|" << pose_set[1] << "|" << pose_set[2] << "|" << pose_set[3] << "|" << pose_set[4] << "|" << pose_set[5] << "|" << pose_set[6] << endl;
  cout << "_kAccMaxTrans: " << _kAccMaxTrans << endl;
  cout << "_kVelMaxTrans: " << _kVelMaxTrans << endl;
  cout << "_kAccMaxRot: " << _kAccMaxRot << endl;
  cout << "_kVelMaxRot: " << _kVelMaxRot << endl;
  cout << "_fHz: " << _fHz << endl;
  RUT::MotionPlanningTrapezodial(pose0, pose_set, _kAccMaxTrans, _kVelMaxTrans,
      _kAccMaxRot, _kVelMaxRot, _fHz, &pose_traj);
  int Nsteps = pose_traj.cols();
  cout << "Nsteps: " << Nsteps << endl;
  cout << "Press Enter to begin motion." << endl;
  getchar();
  ros::Rate pub_rate(_fHz);
  for (int i = 0; i < Nsteps; ++i) {
    cout << "pose_send: ";
    for (int j = 0; j < 7; ++j) {
      pose_send[j] = pose_traj(j, i);
      cout << pose_send[j] << ", ";
    }
    cout << endl;
    _robot->setCartesian(pose_send);
    pub_rate.sleep();
  }
  usleep(500 * 1000); // pause 0.5s
}

int main(int argc, char* argv[]) {
  ROS_INFO_STREAM("Calibrating weight of tool");
  ros::init(argc, argv, "ft_calibration_node");
  ros::NodeHandle hd;

  int Nx, Ny;
  double AngXmax, AngXmin, AngYmax, AngYmin;
  hd.param(std::string("/calibration/Number_of_tilt_x"), Nx, 3);
  if (!hd.hasParam("/calibration/Number_of_tilt_x"))
    ROS_ERROR_STREAM("Parameter [/calibration/Number_of_tilt_x] not found!!!");
  hd.param(std::string("/calibration/Number_of_tilt_y"), Ny, 3);
  if (!hd.hasParam("/calibration/Number_of_tilt_y"))
    ROS_ERROR_STREAM("Parameter [/calibration/Number_of_tilt_y] not found!!!");
  hd.param(std::string("/calibration/Max_rotation_about_x_axis"), AngXmax, 10.0);
  if (!hd.hasParam("/calibration/Max_rotation_about_x_axis"))
    ROS_ERROR_STREAM("Parameter [/calibration/Max_rotation_about_x_axis] not found!!!");
  hd.param(std::string("/calibration/Min_rotation_about_x_axis"), AngXmin, -10.0);
  if (!hd.hasParam("/calibration/Min_rotation_about_x_axis"))
    ROS_ERROR_STREAM("Parameter [/calibration/Min_rotation_about_x_axis] not found!!!");
  hd.param(std::string("/calibration/Max_rotation_about_y_axis"), AngYmax, 10.0);
  if (!hd.hasParam("/calibration/Max_rotation_about_y_axis"))
    ROS_ERROR_STREAM("Parameter [/calibration/Max_rotation_about_y_axis] not found!!!");
  hd.param(std::string("/calibration/Min_rotation_about_y_axis"), AngYmin, 10.0);
  if (!hd.hasParam("/calibration/Min_rotation_about_y_axis"))
    ROS_ERROR_STREAM("Parameter [/calibration/Min_rotation_about_y_axis] not found!!!");

  // control rate
  hd.param(string("/main_loop_rate"), _fHz, 500.0);
  if (!hd.hasParam("/main_loop_rate"))
    ROS_WARN_STREAM("Parameter [/main_loop_rate] not found, using default: " << _fHz);

    // Speed limit for motion planning
  hd.param(string("/vel_max_translation"), _kVelMaxTrans, 0.0);
  hd.param(string("/acc_max_translation"), _kAccMaxTrans, 0.0);
  hd.param(string("/vel_max_rotation"), _kVelMaxRot, 0.0);
  hd.param(string("/acc_max_rotation"), _kAccMaxRot, 0.0);
  if (!hd.hasParam("/vel_max_translation"))
    ROS_WARN_STREAM("Parameter [/vel_max_translation] not found!");
  if (!hd.hasParam("/acc_max_translation"))
    ROS_WARN_STREAM("Parameter [/acc_max_translation] not found!");
  if (!hd.hasParam("/vel_max_rotation"))
    ROS_WARN_STREAM("Parameter [/vel_max_rotation] not found!");
  if (!hd.hasParam("/acc_max_rotation"))
    ROS_WARN_STREAM("Parameter [/acc_max_rotation] not found!");

  Clock::time_point time0 = Clock::now();
  ATINetft ati;
  cout << "[test] initializing ft sensor:\n";
  ati.init(hd, time0);
  cout << "[test] initializing robot:\n";
  // URSocket *ur_robot = URSocket::Instance();
  // ur_robot->init(hd, time0);
  // _robot = ur_robot;
  ABBEGM *abb_robot = ABBEGM::Instance();
  abb_robot->init(hd, time0);
  _robot = abb_robot;

  _ft = &ati;

  /**
   * Calibration begins
   */
  double *ang_x_array = new double[Nx];
  double *ang_y_array = new double[Ny];

  for (int i = 0; i < Nx; ++i)
    ang_x_array[i] = AngXmin + (AngXmax - AngXmin)*double(i)/double(Nx-1);
  for (int i = 0; i < Ny; ++i)
    ang_y_array[i] = AngYmin + (AngYmax - AngYmin)*double(i)/double(Ny-1);

  Eigen::AngleAxisd aaX(0, Vector3d::UnitX());
  Eigen::AngleAxisd aaY(0, Vector3d::UnitY());
  Quaterniond qset(1, 0, 0, 0);

  vector<Vector6d> wrenches;
  vector<Quaterniond> quats;

  double pose[7], wrench[6];
  _robot->getCartesian(pose);

  ros::Rate pub_rate(100);

  Quaterniond q0(pose[3], pose[4], pose[5], pose[6]);
  cout << "Alright! Press Enter to begin.." << endl;
  getchar();

  // begin rotation & data collection
  for (int i = 0; i < Nx; ++i) {
    cout << "[X rotation]: " << ang_x_array[i] << endl;
    aaX.angle() = ang_x_array[i]*PI/180.0;
    for (int j = 0; j < Ny; ++j) {
      cout << "   [Y rotation]: " << ang_y_array[j] << endl;
      aaY.angle() = ang_y_array[j]*PI/180.0;
      Quaterniond qr(aaX*aaY);

      qset    = QuatMTimes(qr, q0);
      pose[3] = qset.w();
      pose[4] = qset.x();
      pose[5] = qset.y();
      pose[6] = qset.z();

      cout << "Pose to be sent:" << endl;
      for (int i = 0; i < 7; ++i) cout << pose[i] << "|";
        cout << endl;

      moveTo(pose);

      cout << "Moving... Press Enter when Done" << endl;
      getchar();

      _ft->getWrenchTool(wrench);
      Vector6d wrenchVec;
      wrenchVec << wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5];
      wrenches.push_back(wrenchVec);
      quats.push_back(qset);
    }
  }

  // move back to original pose
  cout << "[Data collection is done.]" << endl << endl;
  pose[3] = q0.w();
  pose[4] = q0.x();
  pose[5] = q0.y();
  pose[6] = q0.z();
  moveTo(pose);

  /**
   * Motion finished. Begin computation
   * Solve for G, F
   */
  MatrixXd A(Nx*Ny*3, 6);
  MatrixXd b(Nx*Ny*3, 1);
  for (int i = 0; i < Nx*Ny; ++i) {
    A.block<3,3>(3*i, 0) = quats[i].normalized().toRotationMatrix().transpose();
    A.block<3,3>(3*i, 3) = - Matrix3d::Identity();
    b.block<3,1>(3*i, 0) << wrenches[i][0], wrenches[i][1], wrenches[i][2];
  }

  cout << "A: " << endl << A << endl;
  cout << "b: " << endl << b << endl;
  cout << "Solving SVD..." << endl;

  Vector6d x_GF = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  cout << "Done." << endl;

  Vector3d G, F;
  G << x_GF[0], x_GF[1], x_GF[2];
  F << x_GF[3], x_GF[4], x_GF[5];
  cout << "G: " << G << endl << "F: " << F << endl;

  /*
    Solve for P, T
  */
  for (int i = 0; i < Nx*Ny; ++i) {
    double F_hat[3] = {wrenches[i][0] + F[0], wrenches[i][1] + F[1], wrenches[i][2] + F[2]};
    A.block<3,3>(3*i, 0) <<         0,  F_hat[2], -F_hat[1],
    -F_hat[2],          0,  F_hat[0],
    F_hat[1], -F_hat[0],          0;
    b.block<3,1>(3*i, 0) << wrenches[i][3], wrenches[i][4], wrenches[i][5];
  }
  cout << "A: " << endl << A << endl;
  cout << "b: " << endl << b << endl;
  cout << "Solving SVD..." << endl;

  Vector6d x_PT = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  cout << "Done." << endl;

  Vector3d P, T;
  P << x_PT[0], x_PT[1], x_PT[2];
  T << x_PT[3], x_PT[4], x_PT[5];

  Eigen::IOFormat vecFormat(3, 0, "", "\n", "", "", "[", "];");

  VectorXd Foffset(6);
  Foffset << F, T;

  cout << "\n\nCopy the following to your config yaml file:\n\n";
  cout << "------------------- begin of yaml -------------------\n";
  cout << "ftsensor:\n";
  cout << "  offset:\n";
  cout << "    fx: " << Foffset(0) << endl;
  cout << "    fy: " << Foffset(1) << endl;
  cout << "    fz: " << Foffset(2) << endl;
  cout << "    tx: " << Foffset(3) << endl;
  cout << "    ty: " << Foffset(4) << endl;
  cout << "    tz: " << Foffset(5) << endl;
  cout << "  gravity:\n";
  cout << "    x: " <<  G(0) << endl;
  cout << "    y: " <<  G(1) << endl;
  cout << "    z: " <<  G(2) << endl;
  cout << "  COM:\n";
  cout << "    x: " << P(0) << endl;
  cout << "    y: " << P(1) << endl;
  cout << "    z: " << P(2) << endl << endl;
  cout << "------------------- end of yaml -------------------\n";

  delete [] ang_x_array;
  delete [] ang_y_array;

  ROS_INFO_STREAM(endl << "[MAIN] Rest in Peace." << endl);
  return 0;
}
