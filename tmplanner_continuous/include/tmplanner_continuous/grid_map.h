/*
 * Copyright (c) 2018, Ajith Anil Meera, Marija Popovic, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GRID_MAP_H_
#define GRID_MAP_H_

#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <chrono>
#include <thread>   
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <glog/logging.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "minkindr_conversions/kindr_msg.h"
#include "minkindr_conversions/kindr_tf.h"
#include "minkindr_conversions/kindr_xml.h"

#include "tmplanner_continuous/cov_fun.h"
#include "tmplanner_continuous/cov_matern3_iso.h"
#include "tmplanner_continuous/logger.h"

// for pdf and cdf pf normal distribution
#include "tmplanner_continuous/stats/stats.hpp"   

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include "voxblox/core/esdf_map.h"

// for YOLO human detector
// #include <actionlib/client/simple_action_client.h>
// #include "darknet_ros_msgs/CheckForObjectsAction.h"
// //#include "tmplanner_continuous/darknet/include/darknet.h"
// #include "image.h"
// #include "darknet.h"

#include "human_detection/common.hpp"
#include "human_detection/detectors/human_detector.hpp"
#include "human_detection/detectors/human_detector_factory.hpp"
// #include <sensor_msgs/Image.h>


// #include <X11/Xlib.h>
// #ifdef DARKNET_FILE_PATH
// std::string darknetFilePath_ = DARKNET_FILE_PATH;
// #else
// #error Path of darknet repository is not defined in CMakeLists.txt.
// #endif


//#include "matplotlibcpp.h"
//namespace plt = matplotlibcpp;

namespace grid_map {

// Observation submap - sensor footprint.
// upper_left   -------   upper_right
//             |       |
//             |       |
// lower_left   -------   lower_right
//
struct SubmapCoordinates {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d lower_left;
  Eigen::Vector2d upper_right;
  Eigen::Vector2d upper_left;
  Eigen::Vector2d lower_right;
};

class GridMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GridMap(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  GridMap() = default;
  GridMap(const GridMap&) = default;
  GridMap& operator=(const GridMap&) = default;
  ~GridMap() = default;

  // Sets the geometry of the 2D grid map, given:
  // environment dimensions [m], map resolution [m/cell], map origin [m].
  void setMapGeometry(const double& width, const double& height,
                      const double& resolution_x, const double& resolution_y,
                      const double& position_x, const double& position_y,
                      const std::string& frame_id,
                      const std::string sensor_type);

  // Sets unknown values (0.5) for entire grid map.
  void fillUnknown();
  // Manually sets data for entire grid map.
  void fillMapData(const Eigen::MatrixXd& data);
  // Computes the covariance of the map at a resolution with a specified kernel.
  void computeCovariance(const std::string cov_fun,
                         const std::vector<double> log_hyperparams,
                         const double& predict_resolution_x,
                         const double& predict_resolution_y,
                         const std::shared_ptr<voxblox::EsdfMap>& esdf_map_,
                         const std::string cmaes_objective);

  // Computes the trace of the covariance matrix, using a lower threshold on the
  // corresponding mean, if provided.
  double computeCovarianceTrace() const;
  double computeCovarianceTrace(const double& lower_threshold) const;

  // Calculates edge size [m] of an image observed from a height [m].
  Eigen::Vector2d getImageEdgeSize(const double& camera_height,
                                   const double& sensor_fov_angle_x,
                                   const double& sensor_fov_angle_y) const;
  // Calculates the co-ordinates of the observed corners of a submap,
  // assuming camera is perfectly aligned with grid map.
  SubmapCoordinates getRectSubmapCoordinates(
      const geometry_msgs::Point& camera_position,
      const Eigen::Vector2d& image_edge_size) const;

  // Updates the environment grid map based on pixel data in a received image
  // and camera parameters.
  void updateMapFromImage(const cv_bridge::CvImagePtr& image,
                          const geometry_msgs::Pose& mav_pose,
                          const kindr::minimal::QuatTransformation& T_IMU_CAM,
                          const double& sensor_fov_angle_x,
                          const double& sensor_fov_angle_y,
                          const cv::Mat& intrinsics_matrix,
                          const double& sensor_coefficient_A,
                          const double& sensor_coefficient_B,
                          const std::shared_ptr<voxblox::EsdfMap>&,
                          const double width, const double height,
                          const double min_z, const double max_z,
                          const std::string cmaes_objective, 
                          const std::string sensor_type,
                          std::unique_ptr<human_detection::HumanDetector>& human_detector_ptr,
                          const double run_time);

  // Updates the environment grid map covariance based on prediction of what
  // will be seen from a location given the camera parameters.
  void predictMapUpdate(const geometry_msgs::Pose& mav_pose,
                        const kindr::minimal::QuatTransformation& T_IMU_CAM,
                        const double& sensor_fov_angle_x,
                        const double& sensor_fov_angle_y,
                        const double& sensor_coefficient_A,
                        const double& sensor_coefficient_B,
                        const std::shared_ptr<voxblox::EsdfMap>& esdf_map_,
                        const double width, const double height,
                        const double min_z, const double max_z,
                        const std::string cmaes_objective,
                        const bool if_kfupdate);

  // Calculates the KF update for the grid map given a set of measurements
  // (z), their constant noise variance (var), and a (linearized) measurement
  // model (H) using Cholesky factorization.
  void KFUpdate(const Eigen::VectorXd& z, const double& var,
                const Eigen::SparseMatrix<double>& H,
                const std::string cmaes_objective);
  // Performs covariance update only (no measurements) for map update
  // prediction.
  void KFUpdate(const double& var, const Eigen::SparseMatrix<double>& H, const std::string cmaes_objective);

  // Projects a pixel in a camera image to the ground plane based on
  // roll-pitch-yaw angles.
  Eigen::Vector4d projectPixelToGround(
      const cv::Mat& intrinsics_matrix,
      const kindr::minimal::QuatTransformation& T_W_CAM,
      const Eigen::Vector4d& point);

  void setGroundTruth(const std::string sensor_type, const double& width, const double& height);
  void addHumanToGroundTruth(Eigen::Vector2d human_loc, const double& width, const double& height);
  bool insideBoundary(const double width, const double height,
    const double min_z, const double max_z, const geometry_msgs::Point& mav_position);
  bool onGroundPlane(const double width, const double height, const geometry_msgs::Point& ground_point);
  bool isGroundVisible(const geometry_msgs::Point& previous_pose,const geometry_msgs::Point& evaluated_point,
    const std::shared_ptr<voxblox::EsdfMap>& esdf_map_);
  double getDistanceBetweenPoints(const geometry_msgs::Point& p1,const geometry_msgs::Point& p2);
  void addObstaclesGP(const std::shared_ptr<voxblox::EsdfMap>& esdf_map_, const std::string cmaes_objective);
  double computeDetections();
  double minDataWithoutObstacles(const std::shared_ptr<voxblox::EsdfMap>& esdf_map_);
  Eigen::MatrixXd computeSTD();
  void expectedImprovement();
  void probabilityOfImprovement();
  void upperConfidenceBound();
  void detectHuman(cv::Mat& image1, 
      std::unique_ptr<human_detection::HumanDetector>& human_detector_ptr, 
      std::vector<darknet_ros::RosBox_>& rect_points,
      const cv::Mat& intrinsics_matrix,
      const kindr::minimal::QuatTransformation& T_MAP_CAM);
  bool ifInsideRectangle(const darknet_ros::RosBox_& rect_point, const size_t i,const size_t j);
  void removeGiants(
    std::vector<darknet_ros::RosBox_>& rect_points,
    const cv::Mat& intrinsics_matrix,
    const kindr::minimal::QuatTransformation& T_MAP_CAM,
    cv_bridge::CvImagePtr* output_image_ptr);
  void pixelToEnvironment(
    Eigen::Vector2d& point,
    const cv::Mat& intrinsics_matrix,
    const kindr::minimal::QuatTransformation& T_MAP_CAM);
  double calculateRMSE();
  double calculateWRMSE();

  int capture_id;

  // Constructs the measurement model (H) for the KF by matching map states to
  // measurements.
  Eigen::SparseMatrix<double> constructMeasurementModel(
      const Eigen::VectorXd& measurement_indices) const;
  // Calculates the noise variance of an exponentially altitude-dependent
  // sensor.
  double computeSensorNoiseVariance(const double& camera_height,
                                    const double& sensor_coefficient_A,
                                    const double& sensor_coefficient_B) const;

  // Converts co-ordinates from grid map to environment representation.
  void gridToEnvironmentCoordinates(geometry_msgs::Point* point) const;
  // Converts co-ordinates from environment to grid map representation.
  void environmentToGridCoordinates(geometry_msgs::Point* point) const;
  void environmentToGridCoordinates(Eigen::Vector2d* point) const;
  void environmentToGridCoordinates(
      SubmapCoordinates* submap_coordinates) const;
  // Crops submap co-ordinates to nearest valid bounds given the map size.
  void trimSubmapCoordinatesToGrid(SubmapCoordinates* submap_coordinates) const;

  Eigen::MatrixXd getData() const { return data_; }
  Eigen::MatrixXd getCovariance() const { return covariance_; }
  double getAcquisitionView() const { return acquisition_view; }
  double getAcquisitionSum() const { return acquisition.sum(); }
  double getAcquisitionMax() const { return acquisition.maxCoeff(); }
  std::string getFrameId() const { return frame_id_; }
  Eigen::Vector2i getLength() const { return length_; }
  Eigen::Vector2d getPosition() const { return position_; }
  Eigen::Vector2d getResolution() const { return resolution_; }

 private:
  // 2D grid map data.
  Eigen::MatrixXd data_;
  // 2D covariance data.
  Eigen::MatrixXd covariance_;
  // 2D ground truth.
  Eigen::MatrixXd ground_truth;
  // 2D acquisition function
  Eigen::MatrixXd acquisition;
  double acquisition_view;

  // Frame ID of the map.
  std::string frame_id_;
  // Side lengths of the map [cells].
  Eigen::Vector2i length_;
  // Position of origin in the grid map frame [m].
  Eigen::Vector2d position_;
  // Map resolution in each dimension [m/cell].
  Eigen::Vector2d resolution_;

  MappingLogger logger_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
};

}  // namespace grid_map

#endif  // GRID_MAP_H_