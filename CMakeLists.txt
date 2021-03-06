cmake_minimum_required(VERSION 3.4)

project(libfranka-ncs CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(ncs_common STATIC
  ncs_common.cpp
)



target_link_libraries(ncs_common PUBLIC Franka::Franka Eigen3::Eigen3)

set(NCSS
  xyz_follower
  wxyz_follower
  capsense
)

foreach(ncs ${NCSS})
  add_executable(${ncs} ${ncs}.cpp)
  target_link_libraries(${ncs} Franka::Franka ncs_common Eigen3::Eigen3 "/opt/libfranka/lib/libNatNetLibShared.so")
endforeach()
   

include(GNUInstallDirs)
install(TARGETS ${NCSS}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
