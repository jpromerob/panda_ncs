
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
#include <franka/spline.h>

#include <franka/NatNetTypes.h>
#include <franka/NatNetCAPI.h>
#include <franka/NatNetClient.h>

using namespace std;


#pragma pack(1)

typedef struct xyz_payload_t {
    float x;
    float y;
    float z;
    float qx;
    float qy;
    float qz;
    float qw;
} xyz_payload;

typedef struct joints_payload_t {
    double q0;
    double q1;
    double q2;
    double q3;
    double q4;
    double q5;
    double q6;
    
    double x;
    double y;
    double z;

    double qx;
    double qy;
    double qz;
    double qw;
} joints_payload;

joints_payload joints_out;

#pragma pack()


int op_mode;


typedef struct coor
{
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
} coor;

std::mutex mutex_nextcoor;


/* Limits of the workspace*/
coor minimum;
coor maximum;

/* Current Target */
coor c_target;

/* What the optitrack sees (in Panda coordinate framework)*/
coor nextcoor;




/***********************************************************************************************/
/* This function returns a 'final_value' for 'value' limited by the accesible workspace        */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/***********************************************************************************************/
double check_limit(double value, char axis) {

    double final_value = value;
    double max_val = 0.15;
    double min_val = 0.15;
    
    if(axis == 'x') {
      max_val = maximum.x;
      min_val = minimum.x;
    }
    if(axis == 'y') {
      max_val = maximum.y;
      min_val = minimum.y;
    }
    if(axis == 'z') {
      max_val = maximum.z;
      min_val = minimum.z;
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
/* x, y, z are converted from optitrack coordinates to panda coordinates and stored in 'nextcoor'
/*
/*
/*
/*
/***********************************************************************************************/
void save_nextcoor(double x, double y, double z, double qx, double qy, double qz, double qw) {

  double offset_x = 0.0; // offset from object origin to robot end effector
  double offset_y = 0.0;
  double offset_z = 0.0;

  if (mutex_nextcoor.try_lock()) {
    nextcoor.x = check_limit( x + 0.35 + offset_x, 'x'); 
    nextcoor.y = check_limit(-z + 0.36 + offset_y, 'y');
    nextcoor.z = check_limit( y + 0.01 + offset_z, 'z');
    nextcoor.qx = qx;
    nextcoor.qy = qy;
    nextcoor.qz = qz;
    nextcoor.qw = qw;
    mutex_nextcoor.unlock();
  }


}

int t_counter = 0;
double init_t = 0.0;


/***********************************************************************************************/
/* This function just creates a socket for TCP communication                                  */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/***********************************************************************************************/

int createSocket(int port)
{
    int sock, err;
    struct sockaddr_in server;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("ERROR: Socket creation failed\n");
        exit(1);
    }

    bzero((char *) &server, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(port);
    if (bind(sock, (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        printf("ERROR: Bind failed\n");
        exit(1);
    }
    printf("Listening for coordinates\n");

    listen(sock , 3);

    return sock;
}


void sendMsg(int sock, void* msg, uint32_t msgsize)
{
    if (write(sock, msg, msgsize) < 0)
    {
        printf("Can't send message.\n");
        close(sock);
        exit(1);
    }
    return;
}

/***********************************************************************************************/
/* This function is in charge of receiving end-effector position data (x, y, z) through socket */
/* and sending back data related to robot joint positions                                      */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/***********************************************************************************************/
int get_xyz_send_q()
{



    int PORT = 2600;
    int BUFFSIZE = 512;
    char buff[BUFFSIZE];
    int ssock, csock;
    int nread;
    struct sockaddr_in client;
    socklen_t clilen = sizeof(client);

    ssock = createSocket(PORT);
    printf("Server listening on port %d\n", PORT);

    while (1)
    {
        csock = accept(ssock, (struct sockaddr *)&client, &clilen);
        if (csock < 0)
        {
            printf("Error: accept() failed\n");
            continue;
        }

        bzero(buff, BUFFSIZE);
        while ((nread=read(csock, buff, BUFFSIZE)) > 0)
        {
            xyz_payload *p = (xyz_payload*) buff;

            save_nextcoor(p->x, p->y, p->z,
                          p->qx, p->qy, p->qz, p->qw);
            sendMsg(csock, &joints_out, sizeof(joints_payload));
              
        }
        close(csock);
    }

    close(ssock);
    return 0;
}

/***********************************************************************************************/
/* Max and min values in robot coordinate frame                                                */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/***********************************************************************************************/
void init_maxmin_values(){


  minimum.x = -0.20;
  minimum.y = -0.80;
  minimum.z = 0.10;

  maximum.x = 0.60; //
  maximum.y = 0.20; // +y points to munin
  maximum.z = 0.80; // up down

}


/***********************************************************************************************/
/* The next end-effector position should not be too far away from the current one              */
/* When it's too far, the target is modified (restricted) to prevent violent movements         */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/***********************************************************************************************/
double validate(double curr_coor, double next_coor){

    double delta = 0.10; // 10[cm]
    double valid_coor = next_coor;
    if(next_coor > curr_coor + delta){
      valid_coor = curr_coor + delta;
    }
    if(next_coor < curr_coor - delta){
      valid_coor = curr_coor - delta;
    }

    return valid_coor;

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
    double speed = 0.2; // [m/s]
    double force = 140; // [N]

    std::cout << "Grasping ... " << std::endl;
    
    if (!gripper.grasp(object_width, speed, force)) {
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


    if (mutex_nextcoor.try_lock()) {
      nextcoor.x = final_pose[12]; 
      nextcoor.y = final_pose[13];
      nextcoor.z = final_pose[14];

      Eigen::Quaterniond initial_quat(Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.O_T_EE_c.data())).linear());
      nextcoor.qx = initial_quat.x();
      nextcoor.qy = initial_quat.y();
      nextcoor.qz = initial_quat.z();
      nextcoor.qw = initial_quat.w();

      std::cout << "Initial pose: " << nextcoor.x << " | " << nextcoor.y << " | " << nextcoor.z << " || " << nextcoor.qx << " | " << nextcoor.qy << " | " << nextcoor.qz << " | " << nextcoor.qw << std::endl;

      mutex_nextcoor.unlock();
    }

    return false;
  });

}

void limit_vec(Eigen::Vector3d &vec, double limit)
{
  const double vec_norm = vec.norm();
  if(vec_norm > limit)
    vec *= (limit/vec_norm);
}

Eigen::Vector3d limit_vec_calc(const Eigen::Vector3d &vec, double limit)
{
  const double vec_norm = vec.norm();
  if(vec_norm > limit)
    return vec*(limit/vec_norm);
  
  return vec;
}

#define PI 3.14159265



void move_end_effector(char* robot_ip) {

  std::cout << "Are things ready ??? If so, press Enter\n" << std::endl;
  std::cin.ignore();
  
  std::cout << robot_ip << std::endl;  

  // Compliance parameters
  const double translational_stiffness{150.0};
  const double rotational_stiffness{50.0};
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
    franka::RobotState initial_state = robot.readOnce();



    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration /*duration*/) -> franka::Torques {

                            
      mutex_nextcoor.lock();
      c_target.x = validate(robot_state.O_T_EE[12], nextcoor.x);
      c_target.y = validate(robot_state.O_T_EE[13], nextcoor.y);
      c_target.z = validate(robot_state.O_T_EE[14], nextcoor.z);
      c_target.qx = nextcoor.qx;
      c_target.qy = nextcoor.qy;
      c_target.qz = nextcoor.qz;
      c_target.qw = nextcoor.qw;
      mutex_nextcoor.unlock();           
  
      // Reading joints' positions
      joints_out.q0 = robot_state.q[0]; 
      joints_out.q1 = robot_state.q[1]; 
      joints_out.q2 = robot_state.q[2]; 
      joints_out.q3 = robot_state.q[3]; 
      joints_out.q4 = robot_state.q[4]; 
      joints_out.q5 = robot_state.q[5]; 
      joints_out.q6 = robot_state.q[6];

      joints_out.x = robot_state.O_T_EE[12];
      joints_out.y = robot_state.O_T_EE[13];
      joints_out.z = robot_state.O_T_EE[14];
      
      {
        Eigen::Affine3d    initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Quaterniond orientation(initial_transform.linear());
        joints_out.qx = orientation.x();
        joints_out.qy = orientation.y();
        joints_out.qz = orientation.z();
        joints_out.qw = orientation.w();
      }


      {
        Eigen::Matrix4d trans;
        trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
        trans.block<3,3>(0,0) = Eigen::Quaterniond(c_target.qw, c_target.qx, c_target.qy, c_target.qz).toRotationMatrix();
        trans.block<3,1>(0,3) = Eigen::Vector3d(c_target.x, c_target.y, c_target.z);

        // actual end effector positioning command
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        initial_transform = trans;

        initial_state.O_T_EE[ 1] = trans(0,0);
        initial_state.O_T_EE[ 2] = trans(1,0);
        initial_state.O_T_EE[ 3] = trans(2,0);
        initial_state.O_T_EE[ 4] = trans(3,0);

        initial_state.O_T_EE[ 4] = trans(0,1);
        initial_state.O_T_EE[ 5] = trans(1,1);
        initial_state.O_T_EE[ 6] = trans(2,1);
        initial_state.O_T_EE[ 7] = trans(3,1);

        initial_state.O_T_EE[ 8] = trans(0,2);
        initial_state.O_T_EE[ 9] = trans(1,2);
        initial_state.O_T_EE[10] = trans(2,2);
        initial_state.O_T_EE[11] = trans(3,2);

        initial_state.O_T_EE[12] = trans(0,3);
        initial_state.O_T_EE[13] = trans(1,3);
        initial_state.O_T_EE[14] = trans(2,3);
        initial_state.O_T_EE[15] = trans(3,3);
      }

      initial_state.q_d[3] = -2.699157; // first_state.q_d[3]; 
      initial_state.q_d[6] = 0.864497; //first_state.q_d[6]; 


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
      error.head(3) << 1.5*(position - position_d);

      // orientation error
      // "difference" quaternion
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
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

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    // robot.setCollisionBehavior(
    //     {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    //     {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    //     {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
    //     {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});


    double time = 0.0;
    Eigen::Vector3d old_dx(0,0,0);
    Eigen::Vector3d old_x(0,0,0);
    Eigen::Vector3d old_e(0,0,0);
    Eigen::Vector3d ie(0,0,0);
    double speed = 0.0;

    std::array<double, 16> start_pose;

    // define callback for the torque control loop
    std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)>
        cartesian_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {

                            
      mutex_nextcoor.lock();
      c_target.x = validate(robot_state.O_T_EE[12], nextcoor.x);
      c_target.y = validate(robot_state.O_T_EE[13], nextcoor.y);
      c_target.z = validate(robot_state.O_T_EE[14], nextcoor.z);
      c_target.qx = nextcoor.qx;
      c_target.qy = nextcoor.qy;
      c_target.qz = nextcoor.qz;
      c_target.qw = nextcoor.qw;
      mutex_nextcoor.unlock();           
  
      // Reading joints' positions
      joints_out.q0 = robot_state.q[0]; 
      joints_out.q1 = robot_state.q[1]; 
      joints_out.q2 = robot_state.q[2]; 
      joints_out.q3 = robot_state.q[3]; 
      joints_out.q4 = robot_state.q[4]; 
      joints_out.q5 = robot_state.q[5]; 
      joints_out.q6 = robot_state.q[6];

      joints_out.x = robot_state.O_T_EE[12];
      joints_out.y = robot_state.O_T_EE[13];
      joints_out.z = robot_state.O_T_EE[14];
      
      {
        Eigen::Affine3d    initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Quaterniond orientation(initial_transform.linear());
        joints_out.qx = orientation.x();
        joints_out.qy = orientation.y();
        joints_out.qz = orientation.z();
        joints_out.qw = orientation.w();
      }

      double dt = period.toSec();
      //std::cout << "\ndt: " << dt << std::endl;
      
      Eigen::Matrix4d trans;
      trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
      trans.block<3,3>(0,0) = Eigen::Quaterniond(c_target.qw, c_target.qx, c_target.qy, c_target.qz).toRotationMatrix();
      trans.block<3,1>(0,3) = Eigen::Vector3d(c_target.x, c_target.y, c_target.z);

      //const double max_ddx = 0.0000001 * (dt/0.001);
      const double max_ddx   = 100; // 1
      const double max_dx    = 0.01; //0.001;
      const double max_x     = 1e-6; // 1e-6
      const double max_ie = 100.0;

      Eigen::Vector3d e(c_target.x - robot_state.O_T_EE_d[12], c_target.y - robot_state.O_T_EE_d[13], c_target.z - robot_state.O_T_EE_d[14]);
      if(time==0.0)
      {
        old_e = e;
        start_pose = robot_state.O_T_EE_d;
      }

      time += dt;

      Eigen::Vector3d x(0,0,0);
      Eigen::Vector3d dx(0,0,0);
      Eigen::Vector3d ddx(0,0,0);

      Eigen::Vector3d de = (e-old_e)/dt;

      ie += e*dt;
      const double ie_norm = ie.norm();
      if(ie_norm > max_ie)
        ie *= (max_ie/ie_norm);
      else if(ie_norm < -max_ie)
        ie *= (-max_ie/ie_norm);

      if(dt > 0)
      {
        ddx = limit_vec_calc(1000.0 * e + 400.0*de, max_ddx); // + 0.01*ie;

        dx = limit_vec_calc(dx + ddx * 0.001, max_dx);

        x = limit_vec_calc(x + 0.5*(dx + old_dx)*dt, max_x);

        // Eigen::Vector3d dx(x/dt);

        // const double target_speed = dx.norm();
        // const double delta_s = target_speed - speed;
        // if(delta_s > max_ddx)
        // {
        //   dx = dx * ((speed+max_ddx) / target_speed);
        // }
        // else if(delta_s < -max_ddx)
        // {
        //   dx = dx * ((speed-max_ddx) / target_speed);
        // }
        
        // speed = dx.norm();

        // if(speed > max_dx)
        // {
        //   dx = dx * (max_dx / speed);
        //   speed = dx.norm();
        // }
      }
      else
      {
        x *= 0.0;
        speed = 0;
      }

      // std::cout << "e: " << e << std::endl;
      // std::cout << "x: " << x << std::endl;
      std::cout << "\nee: " << ddx.norm() << std::endl;
      std::cout << "de: " << dx.norm() << std::endl;
      std::cout << "ie: " << x.norm() << std::endl;

      std::array<double, 16> new_pose = start_pose;
      // new_pose[0]  = start_pose[];
      // new_pose[1]  = trans(1,0);
      // new_pose[2]  = trans(2,0);
      // new_pose[3]  = trans(3,0);
      
      // new_pose[4]  = trans(0,1);
      // new_pose[5]  = trans(1,1);
      // new_pose[6]  = trans(2,1);
      // new_pose[7]  = trans(3,1);

      // new_pose[8]  = trans(0,2);
      // new_pose[9]  = trans(1,2);
      // new_pose[10] = trans(2,2);
      // new_pose[11] = trans(3,2);

      new_pose[12] = robot_state.O_T_EE_d[12] + x(0);
      new_pose[13] = robot_state.O_T_EE_d[13] + x(1);
      new_pose[14] = robot_state.O_T_EE_d[14] + x(2);
      // new_pose[15] = trans(3,3);

      // if (time >= 10.0) {
      //   std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
      //   return franka::MotionFinished(new_pose);
      // }

      old_dx = dx;
      old_x = x;
      old_e = e;

      return new_pose;
    };

    //robot.control(cartesian_control_callback, franka::ControllerMode::kCartesianImpedance, true, franka::kMaxCutoffFrequency);
    robot.control(impedance_control_callback);

    // std::array<double, 16> initial_pose;
    // double time = 0.0;
    // robot.control([&time, &initial_pose](const franka::RobotState& robot_state,
    //                                      franka::Duration period) -> franka::CartesianPose {
    //   time += period.toSec();
    //   if (time == 0.0) {
    //     initial_pose = robot_state.O_T_EE_c;
    //   }
    //   constexpr double kRadius = 0.3;
    //   double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
    //   double delta_x = kRadius * std::sin(angle);
    //   double delta_z = kRadius * (std::cos(angle) - 1);
    //   std::array<double, 16> new_pose = initial_pose;
    //   new_pose[12] += delta_x;
    //   new_pose[14] += delta_z;
    //   if (time >= 10.0) {
    //     std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
    //     return franka::MotionFinished(new_pose);
    //   }
    //   return new_pose;
    // });


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
  

  char robot_ip[] = "172.16.0.2";
  init_maxmin_values();

  // When the program starts the robot 'moves' to its current position
  // This is to prevent sudden motion
  init_panda_pva(robot_ip);

   

  std::thread coor_process (get_xyz_send_q);


  // close_gripper(robot_ip);
  std::thread mot_process (move_end_effector, robot_ip); 

  
  coor_process.join(); 
  printf("Stopped getting incoming data\n");
  mot_process.join();

  return 0;
}