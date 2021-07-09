cmake_minimum_required(VERSION 2.8.3)
project(tmplanner_continuous)

add_definitions(-std=c++11)
#add_definitions(-DEIGEN_DONT_VECTORIZE=1)
#add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1)

find_package(catkin_simple REQUIRED voxblox_ros voxblox rotors_gazebo_plugins human_detection)
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(darknet_ros REQUIRED darknet_ros_msgs)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# BAYESIAN OPTIMISATION
include_directories(${GAZEBO_INCLUDE_DIRS} include/tmplanner_continuous/bayesopt/include/ include/tmplanner_continuous/bayesopt/include/bayesopt include/tmplanner_continuous/bayesopt/src include/tmplanner_continuous/bayesopt/utils)

add_library(${PROJECT_NAME}_bayesopt include/tmplanner_continuous/bayesopt/src/parameters.cpp
				     include/tmplanner_continuous/bayesopt/src/bayesoptcont.cpp
				     include/tmplanner_continuous/bayesopt/src/bayesoptbase.cpp
				     include/tmplanner_continuous/bayesopt/src/posteriormodel.cpp
				     include/tmplanner_continuous/bayesopt/src/dataset.cpp
				     include/tmplanner_continuous/bayesopt/src/criteria_functors.cpp
				     include/tmplanner_continuous/bayesopt/src/nonparametricprocess.cpp
				     include/tmplanner_continuous/bayesopt/src/criteria_hedge.cpp
				     include/tmplanner_continuous/bayesopt/src/gaussian_process_normal.cpp
				     include/tmplanner_continuous/bayesopt/src/gaussian_process_ml.cpp
				     include/tmplanner_continuous/bayesopt/src/student_t_process_nig.cpp
				     include/tmplanner_continuous/bayesopt/src/student_t_process_jef.cpp
				     include/tmplanner_continuous/bayesopt/src/gaussian_process.cpp
				     include/tmplanner_continuous/bayesopt/src/kernel_functors.cpp
				     include/tmplanner_continuous/bayesopt/src/gaussian_process.cpp
				     include/tmplanner_continuous/bayesopt/src/conditionalbayesprocess.cpp
				     include/tmplanner_continuous/bayesopt/src/gaussian_process_hierarchical.cpp
				     include/tmplanner_continuous/bayesopt/src/kernelregressor.cpp
				     include/tmplanner_continuous/bayesopt/src/gauss_distribution.cpp
				     include/tmplanner_continuous/bayesopt/src/student_t_distribution.cpp
				     include/tmplanner_continuous/bayesopt/src/posterior_empirical.cpp
				     include/tmplanner_continuous/bayesopt/src/mcmc_sampler.cpp
				     include/tmplanner_continuous/bayesopt/src/posterior_mcmc.cpp
				     include/tmplanner_continuous/bayesopt/src/inneroptimization.cpp
				     include/tmplanner_continuous/bayesopt/src/bopt_state.cpp
				     include/tmplanner_continuous/bayesopt/src/mean_functors.cpp
				     include/tmplanner_continuous/bayesopt/src/posterior_fixed.cpp

				     include/tmplanner_continuous/bayesopt/utils/ublas_extra.cpp
				     include/tmplanner_continuous/bayesopt/utils/fileparser.cpp
				     include/tmplanner_continuous/bayesopt/utils/parser.cpp
)


catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(${PROJECT_NAME}_tmplanner_node src/node/tmplanner_node.cpp)
cs_add_library(${PROJECT_NAME}_grid_map src/grid_map.cpp)
cs_add_library(${PROJECT_NAME}_lattice src/lattice.cpp)
cs_add_library(${PROJECT_NAME}_tmplanner src/tmplanner.cpp)

add_subdirectory(src/test)

target_link_libraries(${PROJECT_NAME}_tmplanner_node ${catkin_LIBRARIES} ${PROJECT_NAME}_tmplanner ${PROJECT_NAME}_grid_map ${PROJECT_NAME}_lattice ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_NAME}_bayesopt)


install(TARGETS ${PROJECT_NAME}_bayesopt LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
cs_install()
cs_export()
