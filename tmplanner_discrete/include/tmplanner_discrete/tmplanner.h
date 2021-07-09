#ifndef TMPLANNER_H_
#define TMPLANNER_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <glog/logging.h>
#include <gtest/gtest_prod.h>
#include <libcmaes/cmaes.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/trajectory_sampling.h>
#include <nav_msgs/Path.h>
#include <planning_msgs/PolynomialTrajectory4D.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <Eigen/Eigen>

#include "minkindr_conversions/kindr_msg.h"
#include "minkindr_conversions/kindr_tf.h"
#include "minkindr_conversions/kindr_xml.h"

#include "tmplanner_discrete/grid_map.h"
#include "tmplanner_discrete/lattice.h"
#include "tmplanner_discrete/logger.h"

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/simulation/objects.h"
#include "voxblox_ros/esdf_server.h"
#include "voxblox/core/esdf_map.h"
#include "voxblox/simulation/simulation_world.h"
#include <voxblox/mesh/mesh_integrator.h>
#include <planning_msgs/PlannerService.h>   // for coverage planner client

#include "human_detection/common.hpp"
#include "human_detection/detectors/human_detector.hpp"
#include "human_detection/detectors/human_detector_factory.hpp"

#include "tmplanner_discrete/bayesopt/include/bayesopt/bayesopt.hpp"               // For the C++ API
#include <boost/numeric/ublas/assignment.hpp> // <<= op assigment

namespace tmplanner {
enum OptimizationObjective {
  OPTIMIZATION_OBJECTIVE_INFORMATION,
  OPTIMIZATION_OBJECTIVE_CLASSIFICATION,
  OPTIMIZATION_OBJECTIVE_TIMEVARYING
};

struct MapParameters {
  double width;            // Width of the environment [m]
  double height;           // Height of the environment [m]
  double resolution_x;     // Map resolution [m/cell]
  double resolution_y;     // Map resolution [m/cell]
  double upper_threshold;  // Thresholds for occupancy values (0.0-1.0)
  double lower_threshold;
  std::string frame_id;  // Name of the map reference frame
  kindr::minimal::QuatTransformation
      T_W_MAP;  // Transformation from world to map reference frame
};

struct PlanningParameters {
  int control_poses;  /// Number of control poses for a polynomial
                      /// plan (start point fixed)
  geometry_msgs::Point initial_position;  // Position of first measurement [m]
  double maximum_height;   // Maximum allowable altitude to fly [m]
  double minimum_height;   // Minimum allowable altitude to fly [m]
  double reference_speed;  // References for polynomial path optimization [m/s]
  double reference_acceleration;  // [m/s2]
  double reference_yaw;           // [deg]
  double time_budget;             // Total allocated time budget [s]
  int lattice_min_height_points;  // Number of lattice points at lowest altitude
                                  // level.
  double lattice_height_increment;  // Distance between successive altitude
                                    // levels on the lattice.
};

struct SensorParameters {
  double measurement_frequency;  // Weed classifier frequency [Hz]
  double fov_angle_x;            // Camera field of view angle in x-dir [deg]
  double fov_angle_y;            // Camera field of view angle in y-dir [deg]
  double saturation_height;      // Maximum altitude [m] for receiving valid
                                 // sensor measurements.
  std::vector<double> true_positive_coeffs;      // Polynomial coefficients
                                                 // for true positive classification curve[in
                                                 // descending order]
  std::vector<double> false_negative_coeffs;     // Polynomial
                                                 // coefficients for
                                                 // false negative
                                                 // classification curve
  kindr::minimal::QuatTransformation T_IMU_CAM;  // Transformation from the IMU
                                                 // to camera-sensor-body
                                                 // reference frame
  cv::Mat intrinsics_matrix;                     // Camera intrinsic parameters
  std::string sensor_type;
};

struct OptimizationParameters {
  OptimizationObjective
      optimization_objective;  // 3D grid search objective: information
                               // or classification
  std::string optimization_method, error_metric;
  double cmaes_step_size;
  int cmaes_maximum_fevals;
  int cmaes_offsprings;
};

class tmPlanner: public voxblox::EsdfServer {
 public:
  tmPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  tmPlanner() = default;
  tmPlanner(const tmPlanner&) = delete;
  tmPlanner& operator=(const tmPlanner&) = delete;
  ~tmPlanner() = default;

  // Trajectory generation constant
  static constexpr int kPolynomialCoefficients =
      10;  // Number of polynomial coefficients

  const grid_map::GridMap& getGridMap() const { return grid_map_; }
  const PlanningLogger& getLogger() const { return logger_; }
  OptimizationParameters getOptimizationParameters(){ return optimization_parameters_;}
  const mav_trajectory_generation::Trajectory& getTrajectory() const {
    return trajectory_;
  }
  const std::deque<geometry_msgs::Pose>& getControlPoses() const {
    return control_poses_;
  }
  const SensorParameters& getSensorParameters() {return sensor_parameters_; }

  void setMapParameters(const MapParameters& map_parameters) {
    map_parameters_ = map_parameters;
  }
  void setPlanningParameters(const PlanningParameters& planning_parameters) {
    planning_parameters_ = planning_parameters;
  }
  void setSensorParameters(const SensorParameters& sensor_parameters) {
    sensor_parameters_ = sensor_parameters;
    sensor_parameters_.intrinsics_matrix =
        sensor_parameters.intrinsics_matrix.clone();
  }
  void setOptimizationParameters(
      const OptimizationParameters& optimization_parameters) {
    optimization_parameters_ = optimization_parameters;
  }

  void setOdometryPose(const geometry_msgs::Pose& odometry_pose) {
    odometry_pose_ = odometry_pose;
  }

  // Creates the multi-resolution lattice and grid map required for
  // informative planning.
  void setupPlanner();

  // Checks if budget is spent based on elapsed time.
  bool isBudgetSpent();

  // Creates a plan to take a measurement at a specified initial position.
  // Returns "true" if polynomial trajectory to initial position was created.
  // Returns "false" if MAV is already there.
  bool createInitialPlan();

  // Generates an informative path over a replanning horizon given the current
  // MAV pose.
  bool createNewPlan(const geometry_msgs::Pose& current_pose);
  // Creates a plan to land at current (x,y) position.
  void createLandingPlan();

  // Updates the environment grid map from a received image.
  void updateMap(const geometry_msgs::PoseArray& detections_poses,
                 const geometry_msgs::Pose& odometry_pose);
  void updateMap(const cv_bridge::CvImagePtr& cv_image_ptr,
                 const geometry_msgs::Pose& odometry_pose);

  void initializeObstacleMap();
  void buildWorld(voxblox::FloatingPoint plane_height);
  void updateMapMesh();
  double obstacleCost(double dist, std::string cost_type);
  geometry_msgs::Pose findNextBestInformationGridPointRandom(
    const grid_map::GridMap& simulated_grid_map,
    const geometry_msgs::Pose& previous_pose);
  bool isObstacleFreePath(const geometry_msgs::Point& previous_pose,const geometry_msgs::Point& evaluated_point);
  bool isCollisionFreeTrajectory();

  // Fitness function for optimizing control poses of a polynomial for an
  // informative objective.
  double optimizeControlPoses(const double* x, const int N);

 private:
  // Trajectory generation constants.
  static constexpr int kDimensions = 3;  // Number of dimensions.
  static constexpr int kDerivativetoOptimize =
      mav_trajectory_generation::derivative_order::ACCELERATION;
  static constexpr double kFabianConstant =
      6.5;  // Tuning parameter for polynomial optimization.

  // Parameters.
  MapParameters map_parameters_;
  PlanningParameters planning_parameters_;
  SensorParameters sensor_parameters_;
  OptimizationParameters optimization_parameters_;

  // Environment grid map.
  grid_map::GridMap grid_map_;
  // Multi-resolution lattice for 3-D grid search.
  Lattice lattice_;
  PlanningLogger logger_;

  // Path planned for next horizon.
  std::deque<geometry_msgs::Pose> control_poses_;
  mav_trajectory_generation::Trajectory trajectory_;

  // Current MAV pose based on state estimation.
  geometry_msgs::Pose odometry_pose_;

  // for human detector
  std::unique_ptr<human_detection::HumanDetector> human_detector_ptr;

  // Time passed since the start of the mission.
  double elapsed_time_;
  double initial_run_time;

  planning_msgs::PlannerService coverage_srv; // coverage planner service
  int coverage_point;
  // For esdf maps
  ros::Publisher esdf_gt_mesh_pub_;
  voxblox::EsdfMap::Config config_;
  std::unique_ptr<voxblox::Layer<voxblox::EsdfVoxel> > esdf_gt_;
  //std::shared_ptr<voxblox::EsdfMap> esdf_map_;
  voxblox::SimulationWorld simulation_world_;
  
  std::deque<geometry_msgs::Pose> testing_path; // testing path planner

  // Creates a polynomial trajectory through a list of waypoints.
  void createPolynomialTrajectory();

  // Performs a greedy grid search over a list of candidates to create a
  // sequence of most promising points to visit based on an informative
  // objective.
  bool searchGrid(const geometry_msgs::Pose& initial_pose);

  // Finds the single next best point from a list of candidates based on an
  // informative objective.
  geometry_msgs::Pose findNextBestInformationGridPoint(
      const grid_map::GridMap& simulated_grid_map,
      const geometry_msgs::Pose& previous_pose);
  geometry_msgs::Pose findNextBestClassificationGridPoint(
      const grid_map::GridMap& simulated_grid_map,
      const geometry_msgs::Pose& previous_pose);

  // Optimizes the polynomial path (defined by control poses) using a specified
  // routine.
  void optimizePathOnHorizon();

  // Checks for equality between two points with respect to a tolerance.
  static bool arePointsEqual(const geometry_msgs::Point& p1,
                             const geometry_msgs::Point& p2, const double& tol);
  // Computes the Euclidean distance between two points in space.
  static double getDistanceBetweenPoints(const geometry_msgs::Point& p1,
                                         const geometry_msgs::Point& p2);
};

/* Class to be used for C++-API testing */
class bayesianOpt: public bayesopt::ContinuousModel
{
 public:

  tmplanner::tmPlanner* plan_;

  bayesianOpt(size_t dim,bayesopt::Parameters param, tmplanner::tmPlanner *plan):
    ContinuousModel(dim,param), plan_(plan) { }

  double evaluateSample( const vectord &Xi ) 
  {
    double x[100];
    for (size_t i = 0; i < Xi.size(); ++i)
    {
      x[i] = Xi(i); 
    }
    // return plan_->testFunction(Xi.size(),x,NULL,NULL);
      return plan_->optimizeControlPoses(x,Xi.size());
  };


  bool checkReachability( const vectord &query )
  { return true; };
 
};


}  // namespace tmplanner

#endif  // TMPLANNER_H_