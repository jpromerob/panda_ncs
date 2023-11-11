// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <iterator>
#include <franka/exception.h>
#include <franka/model.h>
#include <stdio.h>
#include <unistd.h>

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
int main(int argc, char** argv) {
  
  try {

    franka::Robot robot("172.16.0.2");

    while(1){

      franka::RobotState state = robot.readOnce();
      franka::Model model(robot.loadModel());
      double x = state.O_T_EE[12];
      double y = state.O_T_EE[13];
      double z = state.O_T_EE[14];
      double b = atan2(-state.O_T_EE[2], sqrt(state.O_T_EE[0]*state.O_T_EE[0] + state.O_T_EE[1]*state.O_T_EE[1]));
      double a = atan2(state.O_T_EE[6], state.O_T_EE[10]);
      double g = atan2(state.O_T_EE[1], state.O_T_EE[0]);

      printf("%.3f | %.3f | %.3f | %.3f | %.3f | %.3f\n", x, y, z, a*180/M_PI, b*180/M_PI, g*180/M_PI);
      sleep(0.5);
    }


  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}