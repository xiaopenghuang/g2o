cmake_minimum_required(VERSION 2.8)
project(g2o_test)

set(CMAKE_BUILD_TYPE Release)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}  "${PROJECT_SOURCE_DIR}/cmake_modules")


find_package(G2O REQUIRED)
if(G2O_FOUND)
   include_directories(${G2O_INCLUDE_DIR})
   message("g2o library find : " ${G2O_INCLUDE_DIR})
endif(G2O_FOUND)

find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#set(G2O_LIBS g2o_core g2o_types_slam3d  g2o_solver_cholmod g2o_solver_csparse)
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )  

add_executable(g2o_test src/main.cpp)
target_link_libraries(g2o_test  csparse ${G2O_LIBS}) 