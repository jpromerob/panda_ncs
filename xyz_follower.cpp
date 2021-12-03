
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
#include <unistd.h>


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






double scale = 0.9; // scaling constant 
double max_v = 1.7 * scale; // velocity
double max_a = 13.0 * scale; // acceleration
// double max_j = 6500.0 * scale; // Jerk
double max_j = 100; // Jerk

typedef struct coor
{
    double x;
    double y;
    double z;

} coor;

std::mutex mutex_optic;
std::mutex mutex_panda;


/* Limits of the workspace*/
coor minimum;
coor maximum;

/* Current Target */
coor c_target;
coor u_target;

/* What the optitrack sees (in Panda coordinate framework)*/
coor optic;

/* Proprioception */
coor p_previous;
coor p_current;
coor p_delta;



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
/* x, y, z are converted from optitrack coordinates to panda coordinates and stored in 'optic'
/*
/*
/*
/*
/***********************************************************************************************/
void save_optic(double x, double y, double z) {

  double offset_x = -0.10;
  double offset_y = 0.20;
  double offset_z = 0.20;

  if (mutex_optic.try_lock()) {
    optic.x = check_limit(x + 0.35 + offset_x, 'x'); // Real Value
    optic.y = check_limit(-z + 0.36 + offset_y, 'y');
    optic.z = check_limit(y + 0.01 + offset_z, 'z');
    mutex_optic.unlock();
  }

}



void _WriteFrame(FILE* fp, sFrameOfMocapData* data);
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server
int ConnectClient();
char getch();

static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;

NatNetClient* g_pClient = NULL;
FILE* g_outputFile;

sNatNetClientConnectParams g_connectParams;

int t_counter = 0;
double init_t = 0.0;





/***********************************************************************************************/
/*                         
/*                                                                                            
/*  Establish a NatNet Client connection                                                                                              
/*                                                                                            
/*                                                                                            
/***********************************************************************************************/
int ConnectClient()
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
        return ErrorCode_Internal;
    }

    return ErrorCode_OK;
}


/***********************************************************************************************/
/*                         
/*                                                                                            
/*  DataHandler receives data from the server                                                                                         
/*                                                                                            
/*                                                                                            
/***********************************************************************************************/
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData)
{

  bool save_and_print = false;

  NatNetClient* pClient = (NatNetClient*) pUserData;

  /* Timestamps start at 0.000 */
  if (t_counter == 0) {
      init_t = data->fTimestamp;
      t_counter++;
  }

  if (save_and_print) {
    if (g_outputFile) {
        _WriteFrame( g_outputFile, data );
    }
  }
    

  int i=0;
	
  // timecode - for systems with an eSync and SMPTE timecode generator - decode to values
	int hour, minute, second, frame, subframe;
  NatNet_DecodeTimecode( data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe );

	
  
  
  for(i=0; i < data->nRigidBodies; i++)
  {
        // params
        // 0x01 : bool, rigid body was successfully tracked in this frame
        bool bTrackingValid = data->RigidBodies[i].params & 0x01;
        if (save_and_print) {
          printf("%3.3lf\t%3.3f\t%3.3f\t%3.3f\n",
              data->fTimestamp-init_t, 
              data->RigidBodies[i].x,
              data->RigidBodies[i].y,
              data->RigidBodies[i].z);
        }
        save_optic(data->RigidBodies[i].x, data->RigidBodies[i].y, data->RigidBodies[i].z);

        
  }




	
}

/***********************************************************************************************/
/*                         
/*                                                                                            
/*  Write t,x,y,z in *.csv file
/*                                                                                            
/*                                                                                            
/***********************************************************************************************/
void _WriteFrame(FILE* fp, sFrameOfMocapData* data)
{
    int i;
    for(i=0; i < data->nRigidBodies; i++)
    {
        fprintf(fp, "%3.3lf,%3.3f,%3.3f,%3.3f\n",
            data->fTimestamp-init_t, 
            data->RigidBodies[i].x,
            data->RigidBodies[i].y,
            data->RigidBodies[i].z);
    }

}


/***********************************************************************************************/
/*                         
/*                                                                                            
/*  Get data ... 
/*                                                                                            
/*                                                                                            
/***********************************************************************************************/
char getch()
{
    char buf = 0;
    termios old = { 0 };

    fflush( stdout );

    if ( tcgetattr( 0, &old ) < 0 )
        perror( "tcsetattr()" );

    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;

    if ( tcsetattr( 0, TCSANOW, &old ) < 0 )
        perror( "tcsetattr ICANON" );

    if ( read( 0, &buf, 1 ) < 0 )
        perror( "read()" );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;

    if ( tcsetattr( 0, TCSADRAIN, &old ) < 0 )
        perror( "tcsetattr ~ICANON" );


    return buf;
}


/***********************************************************************************************/
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/***********************************************************************************************/
int get_opti_data() 
{

    // create NatNet client
    g_pClient = new NatNetClient();

    // set the frame callback handler
    g_pClient->SetFrameReceivedCallback(DataHandler, g_pClient );	// this function will receive data from the server

    g_connectParams.connectionType = kDefaultConnectionType;
    
    g_connectParams.serverAddress = "172.16.222.18";
    g_connectParams.localAddress = "172.16.222.31";

    int iResult;

    // Connect to Motive
    iResult = ConnectClient();
    if (iResult != ErrorCode_OK)
    {
        printf("Error initializing client.  See log for details.  Exiting");
        return 1;
    }
	
	// Create data file for writing received stream into
	const char* szFile = "some_data.csv";

	g_outputFile = fopen(szFile, "w");
	if(!g_outputFile)
	{
		printf("error opening output file %s.  Exiting.", szFile);
		exit(1);
	}


	// Ready to receive marker stream!
	printf("\n\n\nClient is connected to server and listening for data...\n\n\n");
	int c;
	bool bExit = false;
	while(c=getch())
	{
		switch(c)
		{
			case 'q':
        printf("Asked to Stop\n");
				bExit = true;		
				break;	
			default:
				break;
		}
		if(bExit)
			break;
	}

	// Done - clean up.
	if (g_pClient)
	{
		g_pClient->Disconnect();
		delete g_pClient;
		g_pClient = NULL;
	}

	if (g_outputFile)
	{
		fclose(g_outputFile);
		g_outputFile = NULL;
	}

	return ErrorCode_OK;  

}



/***********************************************************************************************/
/*                                                                                             */
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
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/***********************************************************************************************/
void read_csv()
{
  vector<string> row;
  ifstream fin;
  string line, word;
  // Open an existing file
  fin.open("examples/FakeOptiTrack.csv");
  while(!fin.eof()){
    getline(fin, line);
    stringstream s(line);
    while (getline (s, word, ',')) {
        row.push_back (word);
    }
    cout << " x: " << row[0] << " y: " << row[1] << " z: " << row[2] << "\n";
  }
}

/***********************************************************************************************/
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/*                                                                                             */
/***********************************************************************************************/
void read_pose(char* robot_ip) {
    franka::Robot robot(robot_ip);
    while (1) {
      robot.read([](const franka::RobotState& robot_state) {

        std::array<double, 16> final_pose;
        std::array<double, 7> final_joints;
        final_pose = robot_state.O_T_EE_c;
        final_joints = robot_state.q_d;

        std::cout << "x: " << final_pose[12] << " | ";
        std::cout << "y: " << final_pose[13] << " | ";
        std::cout << "z: " << final_pose[14] << " | ";
        std::cout << "alpha: " << final_joints[6]*180/M_PI << std::endl;

        return false;
      });
      usleep(100000);
    }
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


/* A new target get approved if it's different from current position by at >1cm in at least one axis */
bool target_approval(double x_i, double y_i, double z_i, double x_f, double y_f, double z_f) {

  bool flag = false;
  double delta = 0.010;

  if((abs(x_i-x_f) > delta) || (abs(y_i-y_f) > delta) || (abs(z_i-z_f) > delta)) {
    flag = true;
  }


  return flag;
}


void init_panda_pva(coor actual){

  p_previous.x = actual.x;
  p_previous.y = actual.y;
  p_previous.z = actual.z;

  p_current.x = actual.x;
  p_current.y = actual.y;
  p_current.z = actual.z;

  p_delta.x = 0.0;
  p_delta.y = 0.0;
  p_delta.z = 0.0;



}

void update_panda_pva(coor actual, double p) {

  /* Update 'previous' */
  p_previous.x = p_current.x;
  p_previous.y = p_current.y;
  p_previous.z = p_current.z;

  /* Update 'current' */
  p_current.x = actual.x;
  p_current.y = actual.y;
  p_current.z = actual.z;

}




/* ct: current target ; ut: updated target*/
void get_new_pose(bool rethink_motion, double * x, double * y, double * z, coor ct, coor ut, coor actual, double t, double p) {


  double scalator = 120;  

  mutex_panda.lock();

  update_panda_pva(actual, p);


  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;

  if (!rethink_motion) {

    dx = tanh((ct.x - p_current.x) / scalator * min(t,1.0));
    dy = tanh((ct.y - p_current.y) / scalator * min(t,1.0));
    dz = tanh((ct.z - p_current.z) / scalator * min(t,1.0));

    p_delta.x = dx;
    p_delta.y = dy;
    p_delta.z = dz;

  } else {

  }

  *x = actual.x + p_delta.x;
  *y = actual.y + p_delta.y;
  *z = actual.z + p_delta.z;

  mutex_panda.unlock();


}

#define PI 3.14159265


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

                                          
      mutex_optic.lock();
      c_target.x = optic.x;
      c_target.y = optic.y;
      c_target.z = optic.z;
      mutex_optic.unlock();                                     


      initial_state.O_T_EE[12] = c_target.x;
      initial_state.O_T_EE[13] = c_target.y;
      initial_state.O_T_EE[14] = c_target.z;


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
      error.head(3) << position - position_d;

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



  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  init_maxmin_values();


  std::thread first (get_opti_data);      
  std::thread second (move_end_effector, argv[1]);  


  std::cout << "Threads running...\n";

  // synchronize threads:
  first.join();                // pauses until first finishes
  second.join();               // pauses until second finishes
  


  return 0;
}