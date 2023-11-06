
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <inttypes.h>
#include <stdio.h>
#include <termios.h>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <mutex>
#include <cstdio>
#include <algorithm>
#include <stdlib.h>
#include <thread>    
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <unistd.h>    //write




#include "ncs_common.h"

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/robot.h>

using namespace std;


#pragma pack(1)

typedef struct payload_t {
    float x;
    float y;
    float z;
    float a;
    float b;
    float g;
} payload;


#define OFFSET_X 0.050
#define OFFSET_Y 0.150
#define OFFSET_Z 0.150

#define MIN_X -0.20
#define MIN_Y -0.80
#define MIN_Z 0.10
#define MAX_X 0.60
#define MAX_Y 0.20
#define MAX_Z 0.80


#pragma pack()


int op_mode;


typedef struct pose
{
    double x;
    double y;
    double z;
    double a;
    double b;
    double g;

} pose;

std::mutex mutex_nextpose;

/* Current Target */
pose c_target;

/* What the optitrack sees (in Panda posedinate framework)*/
pose nextpose;




/***********************************************************************************************/
/* This function returns a 'final_value' for 'value' limited by the accesible workspace        */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/***********************************************************************************************/
double check_xyz_lim(double value, char axis) {

    double final_value = value;
    double max_val = 0.15;
    double min_val = 0.15;
    
    if(axis == 'x') {
      max_val = MAX_X;
      min_val = MIN_X;
    }
    if(axis == 'y') {
      max_val = MAX_Y;
      min_val = MIN_Y;
    }
    if(axis == 'z') {
      max_val = MAX_Z;
      min_val = MIN_Z;
    }

    // Limit Max Val for 'axis'
    if (final_value > max_val) {
      final_value = max_val;
    }

    // Limit Min Val for 'axis'
    if (final_value < min_val) {
      final_value = min_val;
    }

    // std::cout << "Target " << axis << ": " << final_value << std::endl;

    return final_value;
}


/***********************************************************************************************/
/* This function returns a 'final_value' for 'value' limited by -pi..pi range                  */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/***********************************************************************************************/
double check_angle_lim(double value){
  double final_value = value;
  return final_value;
}

#define BASE_G -90.0*M_PI/180
#define BASE_B -30.0*M_PI/180
#define BASE_A 180.0*M_PI/180

/***********************************************************************************************/
/* x, y, z are converted from optitrack posedinates to panda posedinates and stored in 'nextpose'
/*
/*
/*
/*
/***********************************************************************************************/
void save_nextpose(double x, double y, double z, double a, double b, double g) {

  double delta_pitch = 0; 
  double delta_roll = 0 ; 
  double delta_yaw = 0; 

  if (mutex_nextpose.try_lock()) {
    nextpose.x = check_xyz_lim( x + 0.35 + OFFSET_X, 'x'); 
    nextpose.y = check_xyz_lim(-z + 0.36 + OFFSET_Y, 'y');
    nextpose.z = check_xyz_lim( y + 0.01 + OFFSET_Z, 'z');

    delta_roll = atan((OFFSET_X*2-nextpose.x)/OFFSET_Y);

    nextpose.a = check_angle_lim(BASE_A + delta_yaw); // yaw --> alpha : around z (theory)
    nextpose.b = check_angle_lim(BASE_B + delta_pitch); // pitch --> beta : around y (theory) up-down (hand)
    nextpose.g = check_angle_lim(BASE_G + delta_roll); // roll --> gamma : around x (theory) left-right (hand)
    mutex_nextpose.unlock();
  }

  // printf("%.3f | %.3f | %.3f \n", a*180/M_PI, b*180/M_PI, g*180/M_PI);

}





/***********************************************************************************************/
/* This function is in charge of receiving data (x, y, z predicted by SNN) through tcp socket  */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/***********************************************************************************************/
int get_xyz_data()
{
    int PORT = 2600;


    // Create a UDP socket
    int sock;
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket");
        return 1;
    }

    // Bind the socket to a specific address and port
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(PORT);  // Make sure the port matches the Python script
    server_address.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (struct sockaddr *)&server_address, sizeof(server_address)) == -1) {
        perror("bind");
        return 1;
    }

    // Receive and print the data
    payload receivedPayload;
    struct sockaddr_in client_address;
    socklen_t addr_len = sizeof(client_address);

    while(1){

      if (recvfrom(sock, &receivedPayload, sizeof(receivedPayload), 0, (struct sockaddr *)&client_address, &addr_len) == -1) {
          perror("recvfrom");
          return 1;
      }
      save_nextpose(receivedPayload.x, receivedPayload.y, receivedPayload.z,
                    receivedPayload.a, receivedPayload.b, receivedPayload.g);

    }


    return 0;
}



double validate(double curr_pose, double next_pose){

    double delta = 0.10; // 10[cm]
    double valid_pose = next_pose;
    if(next_pose > curr_pose + delta){
      valid_pose = curr_pose + delta;
    }
    if(next_pose < curr_pose - delta){
      valid_pose = curr_pose - delta;
    }

    return valid_pose;

}

/***********************************************************************************************/
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/***********************************************************************************************/
void close_gripper(char * robot_ip) {

    franka::Robot robot(robot_ip);
    franka::Gripper gripper(robot_ip);
    gripper.homing();
    

    // Grasp the object: speed + force
    double object_width = 3/1000.0; // [m]
    double gripper_speed = 0.2; // [m/s]
    double force = 140; // [N]

    std::cout << "Grasping ... " << std::endl;
    
    if (!gripper.grasp(object_width, gripper_speed, force)) {
      std::cout << "Failed to grasp object." << std::endl;
    }

}


void init_panda_pva(char* robot_ip) {

  franka::Robot robot(robot_ip);
  robot.read([](const franka::RobotState& robot_state) {

    std::array<double, 16> final_pose;
    std::array<double, 7> final_joints;
    final_pose = robot_state.O_T_EE_c;
    final_joints = robot_state.q_d;


    if (mutex_nextpose.try_lock()) {
      nextpose.x = final_pose[12]; 
      nextpose.y = final_pose[13];
      nextpose.z = final_pose[14];
      mutex_nextpose.unlock();
    }

    return false;
  });

}

#define PI 3.14159265

double speed = 0;

void move_end_effector(char* robot_ip) {

  std::cout << "Are things ready ???" << std::endl;
  std::cin.ignore();
  
  std::cout << robot_ip << std::endl;  

  // Compliance parameters
  const double translational_stiffness{150.0};
  const double rotational_stiffness{10.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);


  try {

    // connect to robot
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState first_state = robot.readOnce();
    
    franka::RobotState initial_state = first_state;



    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration /*duration*/) -> franka::Torques {

                            
      mutex_nextpose.lock();
      c_target.x = validate(robot_state.O_T_EE[12], nextpose.x);
      c_target.y = validate(robot_state.O_T_EE[13], nextpose.y);
      c_target.z = validate(robot_state.O_T_EE[14], nextpose.z); 
      c_target.a = nextpose.a; // yaw --> alpha : around z (theory)
      c_target.b = nextpose.b; // pitch --> beta : around y (theory)
      c_target.g = nextpose.g; // roll --> gamma : around x (theory)
      mutex_nextpose.unlock();                                    

      // Smooth robot replacement upon program start
      if(speed < 4){
        speed = speed + 0.001;
      }


      // actual end effector positioning command
      initial_state.O_T_EE[0] = cos(c_target.b)*cos(c_target.g);
      initial_state.O_T_EE[1] = cos(c_target.b)*sin(c_target.g);
      initial_state.O_T_EE[2] = -sin(c_target.b);
      initial_state.O_T_EE[4] = sin(c_target.a)*sin(c_target.b)*cos(c_target.g)-cos(c_target.a)*sin(c_target.g);
      initial_state.O_T_EE[5] = sin(c_target.a)*sin(c_target.b)*sin(c_target.g)+cos(c_target.a)*cos(c_target.g);
      initial_state.O_T_EE[6] = sin(c_target.a)*cos(c_target.b);
      initial_state.O_T_EE[8] = cos(c_target.a)*sin(c_target.b)*cos(c_target.g)+sin(c_target.a)*sin(c_target.g);
      initial_state.O_T_EE[9] = cos(c_target.a)*sin(c_target.b)*sin(c_target.g)-sin(c_target.a)*cos(c_target.g);
      initial_state.O_T_EE[10] = cos(c_target.a)*cos(c_target.b);
      initial_state.O_T_EE[12] = c_target.x;
      initial_state.O_T_EE[13] = c_target.y;
      initial_state.O_T_EE[14] = c_target.z;
      initial_state.O_T_EE[15] = 1;


      // equilibrium point is the initial position
      Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
      Eigen::Vector3d position_d(initial_transform.translation());
      Eigen::Quaterniond orientation_d(initial_transform.linear());

      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());


      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << speed*(position - position_d);

      // orientation error
      // "difference" quaternion
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -0.5*orientation.coeffs();
      }
      // "difference" quaternion
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << -transform.linear() * error.tail(3);

      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);

      // Spring damper system with damping ratio=1
      tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
      tau_d << tau_task + coriolis;

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
      return tau_d_array;
    };

    robot.control(impedance_control_callback);


  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

}


/***********************************************************************************************/
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/***********************************************************************************************/


int main(int argc, char** argv) {

  op_mode = 0; // 0: only reading SNN data, 1: closed loop SNN, 2: closed loop optitrack

  std::cout << "Hello NCS people :)\n";
  

  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-ip> <operating-mode>" << std::endl;
    return -1;
  }
  
  
  op_mode = stoi(argv[2]);
  printf("Mode: %d\n", op_mode);

  // When the program starts the robot 'moves' to its current position
  // This is to prevent sudden motion
  init_panda_pva(argv[1]);

  // close_gripper(argv[1]);
   

  switch(op_mode){

    // This mode is only for reading incoming data (from sleipner or else ... through tcp)
    case 0:
      {
        std::thread snn_process (get_xyz_data);
        snn_process.join(); 
        printf("Stopped getting incoming data\n");
        break;
      }
      

    // This mode is for reading incoming data (from sleipner or else ... through tcp) AND making the robot follow
    case 1:
      {
        std::thread snn_process (get_xyz_data);
        std::thread mot_process (move_end_effector, argv[1]);  
        snn_process.join(); 
        printf("Stopped getting incoming data\n");
        mot_process.join();
        break;
      }

    default:
      {
        std::thread mot_process (move_end_effector, argv[1]);  
        mot_process.join();
        printf("Operating mode unavailable\n");
        break;
      }

  }

  return 0;
}