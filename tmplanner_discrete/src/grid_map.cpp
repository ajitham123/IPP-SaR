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

#include "tmplanner_discrete/grid_map.h"

using namespace grid_map;

GridMap::GridMap(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
      nh_(nh),
      nh_private_(nh_private) {};

void GridMap::setMapGeometry(const double& width, const double& height,
                             const double& resolution_x,
                             const double& resolution_y,
                             const double& position_x, const double& position_y,
                             const double& upper_threshold,
                             const double& lower_threshold,
                             const std::string& frame_id,
                             const std::string sensor_type) {
  CHECK_GT(resolution_x, 0) << "Error in map resolution value in x.";
  CHECK_GT(resolution_y, 0) << "Error in map resolution value in y.";
  CHECK_GE(width, 0) << "Error in map width value.";
  CHECK_GE(height, 0) << "Error in map height value.";
  CHECK_LE(upper_threshold, 1.0) << "Error in map threshold values";
  CHECK_LE(lower_threshold, 1.0) << "Error in map threshold values";
  CHECK_GE(upper_threshold, 0.0) << "Error in map threshold values";
  CHECK_GE(lower_threshold, 0.0) << "Error in map threshold values";

  frame_id_ = frame_id;
  resolution_(0) = resolution_x;
  resolution_(1) = resolution_y;
  length_(0) = static_cast<int>(round(height / resolution_x));
  length_(1) = static_cast<int>(round(width / resolution_y));

  data_.resize(length_(0), length_(1));

  // Center the map corners.
  position_(0) = position_x;
  position_(1) = position_y;
  capture_id = 0;

  setGroundTruth(sensor_type,width,height);
  logger_.writeToCSV(ground_truth,"ground_truth");
}

void GridMap::setGroundTruth(const std::string sensor_type, const double& width, const double& height)
{
  if(sensor_type == "human")
  {
    ground_truth.resize(length_(0), length_(1));
    ground_truth.setZero(length_(0), length_(1));
    // ground_truth.setOnes(length_(0), length_(1));
    // ground_truth = ground_truth*0.1;

    Eigen::Vector2d human_loc;
    human_loc[0] = 8;    human_loc[1] = -12;
    addHumanToGroundTruth(human_loc,width,height);
    human_loc[0] = -6;    human_loc[1] = -7;
    addHumanToGroundTruth(human_loc,width,height);
    human_loc[0] = 12;    human_loc[1] = -7;
    addHumanToGroundTruth(human_loc,width,height);      
    human_loc[0] = -7;    human_loc[1] = -12;
    addHumanToGroundTruth(human_loc,width,height);
    human_loc[0] = 4;    human_loc[1] = -11;
    addHumanToGroundTruth(human_loc,width,height);
    human_loc[0] = -12;    human_loc[1] = -6;
    addHumanToGroundTruth(human_loc,width,height);
    human_loc[0] = 0;    human_loc[1] = -7;
    addHumanToGroundTruth(human_loc,width,height); 
  }
}

void GridMap::addHumanToGroundTruth(Eigen::Vector2d human_loc, const double& width, const double& height)
{
  double human_width = 0.75, human_height = 2;
  // Report if humans are placed at the boundaries
  if(human_loc[0]>0.5*(height-human_width-0.1) || human_loc[0]<-0.5*(height-human_width-0.1) || \
     human_loc[1]>0.5*width-human_height-0.1 || human_loc[1]<-0.5*width+0.1)
  {
    LOG(ERROR)<<"Human close to the boundary: ground truth error! ";
    std::exit(0);
  }
  Eigen::Vector2d bottom_left, top_right;
  bottom_left[0] = human_loc[0] - 0.5*human_width;
  bottom_left[1] = human_loc[1] ;
  top_right[0] = human_loc[0] +0.5*human_width;
  top_right[1] = human_loc[1] + human_height;
  environmentToGridCoordinates(&top_right);
  environmentToGridCoordinates(&bottom_left);
  for(int i=bottom_left[0];i<=top_right[0];i++)
  {
    for(int j=bottom_left[1];j<=top_right[1];j++)
    {
      ground_truth(j,i) = 1;
      // LOG(INFO)<<"Ground truth (i,j) "<<i<<","<<j;
    }
  }
}

// void GridMap::fillUnknown() { data_.setZero(length_(0), length_(1)); }

void GridMap::fillUnknown() 
{ 
  map_prior = 0.1;
  data_.setOnes(length_(0), length_(1)); 
  data_ = data_*std::log(map_prior/(1-map_prior)); 
}

void GridMap::fillFree() {
  data_.setOnes(length_(0), length_(1));
  data_ = data_ * -std::numeric_limits<double>::infinity();
}

void GridMap::fillMapData(const Eigen::MatrixXd& data) {
  CHECK_EQ(length_(0), data.rows()) << "Error in GridMap::fillMapData! Data "
                                       "dimensions don't match map dimensions.";
  CHECK_EQ(length_(1), data.cols()) << "Error in GridMap::fillMapData! Data "
                                       "dimensions don't match map dimensions.";
  data_ = data;
}

void GridMap::addObstaclesToMap(const std::shared_ptr<voxblox::EsdfMap>& esdf_map_)
{
  LOG(INFO)<<"Grid length: "<<length_(0)<<","<<length_(1);
  for(int i=0;i<length_(0);i++)
  {
    for(int j=0;j<length_(1);j++)
    {
      geometry_msgs::Point grid_ij;
      grid_ij.x = i;
      grid_ij.y = j;
      grid_ij.z = 0.5;  // obstacles in the horizontal plane above ground cannot be mapped
      gridToEnvironmentCoordinates(&grid_ij);
      Eigen::Vector3d grid_vec; // to deal with data type bullshit
      grid_vec[0] = grid_ij.x;
      grid_vec[1] = grid_ij.y;
      grid_vec[2] = grid_ij.z; 
      double dist;
      if(esdf_map_->getDistanceAtPosition(grid_vec,&dist))
        {
          if(dist<=0) // Change this to variables
          {
            data_(j,i) = std::log(0.02/(1-0.02));
          }
        }
      else
        LOG(ERROR)<<"Point ("<<grid_vec[0]<<","<<grid_vec[1]<<","<<grid_vec[2]<<") not found in the map! Inside addObstaclesToMap() ";
    }
  }

  // Convert data to probability and save
  Eigen::MatrixXd data_prob, entropy_dist;
  data_prob.resize(length_(0), length_(1));
  entropy_dist.resize(length_(0), length_(1));
  for (auto i = 0; i < data_.rows(); ++i) 
    for (auto j = 0; j < data_.cols(); ++j) 
    {
      data_prob(i,j) = 1.0 - (1.0 / (1.0 + std::exp(data_(i, j))));
      entropy_dist(i,j) = -(data_prob(i,j) * std::log(data_prob(i,j)) + (1.0 - data_prob(i,j)) * std::log(1.0 - data_prob(i,j)));
    }

  logger_.writeToCSV(data_prob,"data_");
  logger_.writeToCSV(entropy_dist,"entropy_");
}

double GridMap::computeEntropy() const {
  double entropy = 0;
  for (auto i = 0; i < data_.rows(); ++i) {
    for (auto j = 0; j < data_.cols(); ++j) {
      double p = toProbability(data_(i, j));
      // Set unknown occupancy.
      if (p == -1.0 / 100) {
        p = 0.5;
      }
      CHECK_GT(p, 0) << "Invalid probability in map cell.";
      CHECK_LT(p, 1) << "Invalid probability in map cell.";
      entropy += -(p * std::log(p) + (1.0 - p) * std::log(1.0 - p));
    }
  }
  return entropy;
}

int GridMap::computeNumberOfUnclassifiedPoints() const {
  int number_of_unclassified_points = 0;
  for (auto i = 0; i < data_.rows(); ++i) {
    for (auto j = 0; j < data_.cols(); ++j) {
      double p = toProbability(data_(i, j));
      if ((p > lower_threshold_ && p < upper_threshold_) || p == -1.0 / 100)
        number_of_unclassified_points++;
    }
  }
  return number_of_unclassified_points;
}

Eigen::Vector2d GridMap::getImageEdgeSize(
    const double& camera_height, const double& sensor_fov_angle_x,
    const double& sensor_fov_angle_y) const {
  CHECK_GT(camera_height, 0.0) << "Camera height is lower than 0.0m.";
  Eigen::Vector2d image_edge_size;
  image_edge_size(0) =
      (2.0 * camera_height) * tan((sensor_fov_angle_x / 2.0) * (M_PI / 180.0));
  image_edge_size(1) =
      (2.0 * camera_height) * tan((sensor_fov_angle_y / 2.0) * (M_PI / 180.0));
  return image_edge_size;
}

SubmapCoordinates GridMap::getRectSubmapCoordinates(
    const geometry_msgs::Point& camera_position,
    const Eigen::Vector2d& image_edge_size) const {
  SubmapCoordinates submap_coordinates;
  submap_coordinates.lower_left(0) =
      camera_position.x - (image_edge_size(0) / 2.0);
  submap_coordinates.lower_left(1) =
      camera_position.y - (image_edge_size(1) / 2.0);
  submap_coordinates.upper_right(0) =
      camera_position.x + (image_edge_size(0) / 2.0);
  submap_coordinates.upper_right(1) =
      camera_position.y + (image_edge_size(1) / 2.0);
  submap_coordinates.upper_left(0) =
      camera_position.x - (image_edge_size(0) / 2.0);
  submap_coordinates.upper_left(1) =
      camera_position.y + (image_edge_size(1) / 2.0);
  submap_coordinates.lower_right(0) =
      camera_position.x + (image_edge_size(0) / 2.0);
  submap_coordinates.lower_right(1) =
      camera_position.y - (image_edge_size(1) / 2.0);
  return submap_coordinates;
}

void GridMap::updateMapFromPoseArray(
    const geometry_msgs::PoseArray& detections_poses,
    const geometry_msgs::Pose& mav_pose,
    const kindr::minimal::QuatTransformation& T_IMU_CAM,
    const double& sensor_fov_angle_x, const double& sensor_fov_angle_y,
    const double& saturation_height, const cv::Mat& intrinsics_matrix,
    const std::vector<double>& true_positive_coeffs,
    const std::vector<double>& false_negative_coeffs) {

  LOG(INFO)<<"<detection pose "<< detections_poses.poses[0].position;

  // Obtain world to IMU (vicon-sensor-body) transform.
  kindr::minimal::QuatTransformation T_MAP_IMU;
  tf::poseMsgToKindr(mav_pose, &T_MAP_IMU);
  // Compute map to camera transform.
  kindr::minimal::QuatTransformation T_MAP_CAM = T_MAP_IMU * T_IMU_CAM;
  geometry_msgs::Pose camera_pose;
  tf::poseKindrToMsg(T_MAP_CAM, &camera_pose);

  // Calculate the corner co-ordinates of the observed submap.
  // NB: - Assume that principal point is at the center of the image.
  SubmapCoordinates submap_coordinates;
  Eigen::Vector4d p1;
  p1 << 0.0, 0.0, 1.0, 1.0;
  Eigen::Vector4d p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.upper_left = p2.head(2);
  p1 << 0.0, (intrinsics_matrix.at<double>(1, 2) * 2.0 - 1.0), 1.0, 1.0;
  p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.lower_left = p2.head(2);
  p1 << (intrinsics_matrix.at<double>(0, 2) * 2.0 - 1.0),
      (intrinsics_matrix.at<double>(1, 2) * 2.0 - 1.0), 1.0, 1.0;
  p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.lower_right = p2.head(2);
  p1 << (intrinsics_matrix.at<double>(0, 2) * 2.0 - 1.0), 0.0, 1.0, 1.0;
  p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.upper_right = p2.head(2);
  environmentToGridCoordinates(&submap_coordinates);
  trimSubmapCoordinatesToGrid(&submap_coordinates);

  // Initialize variables to keep track of submap data.
  const size_t x_min = std::min(
      {submap_coordinates.lower_left(0), submap_coordinates.upper_left(0)});
  const size_t x_max = std::max(
      {submap_coordinates.upper_right(0), submap_coordinates.lower_right(0)});
  const size_t y_min = std::min(
      {submap_coordinates.lower_left(1), submap_coordinates.lower_right(1)});
  const size_t y_max = std::max(
      {submap_coordinates.upper_left(1), submap_coordinates.upper_right(1)});
  const size_t submap_size_x = x_max - x_min + 1;
  const size_t submap_size_y = y_max - y_min + 1;
  Eigen::MatrixXd submap;
  submap.setZero(submap_size_y, submap_size_x);

  // Project each tag onto submap, and set corresponding cell to "1".
  for (size_t i = 0; i < detections_poses.poses.size(); ++i) {
    // Get camera to tag transform for this tag.
    kindr::minimal::QuatTransformation T_CAM_TAG;
    tf::poseMsgToKindr(detections_poses.poses[i], &T_CAM_TAG);
    // Compute world to tag transform.
    kindr::minimal::QuatTransformation T_MAP_TAG = T_MAP_CAM * T_CAM_TAG;
    geometry_msgs::Pose tag_pose;
    tf::poseKindrToMsg(T_MAP_TAG, &tag_pose);
    // Determine occupied cell.
    geometry_msgs::Point tag_position_grid = tag_pose.position;
    environmentToGridCoordinates(&tag_position_grid);
    // Skip measurement: out-of-bounds.
    if (tag_position_grid.x - x_min < 0 ||
        tag_position_grid.x - x_min > (submap_size_x - 1) ||
        tag_position_grid.y - y_min < 0 ||
        tag_position_grid.y - y_min > (submap_size_y - 1) ||
        tag_position_grid.x < 0 || tag_position_grid.y < 0 ||
        tag_position_grid.x > (length_(0) - 1) ||
        tag_position_grid.y > (length_(1) - 1)) {
      continue;
    }
    LOG(INFO) << "Detection at x = " << tag_pose.position.x
              << ", y = " << tag_pose.position.y << ".";
    submap(tag_position_grid.y - y_min, tag_position_grid.x - x_min) = 1.0;
  }

  // Perform occupancy map update for each cell in the submap.
  for (size_t i = 0; i < submap.rows(); ++i) {
    for (size_t j = 0; j < submap.cols(); ++j) {
      // Occupancy probability given the current measurement.
      double p;
      // Occupied: true positive sensor model.
      if (submap(i, j) == 1) {
        p = computeTruePositiveProbability(
            camera_pose.position.z, saturation_height, true_positive_coeffs);
        LOG(INFO)<<"updated data from "<<data_(y_min + i, x_min + j)<<" to "<<data_(y_min + i, x_min + j)+std::log(p / (1 - p)) ;
        // Free: false positive sensor model.
      } else {
        p = computeFalseNegativeProbability(
            camera_pose.position.z, saturation_height, false_negative_coeffs);
      }

      data_(y_min + i, x_min + j) += std::log(p / (1 - p));
    }
  }

  // Convert data to probability and save
  Eigen::MatrixXd data_prob;
  data_prob.resize(length_(0), length_(1));
  for (auto i = 0; i < data_.rows(); ++i) 
    for (auto j = 0; j < data_.cols(); ++j) 
      data_prob(i,j) = toProbability(data_(i, j));

  std::ofstream f1;
  f1.open(ros::package::getPath("tmplanner_discrete") + "/debug/data_.csv");
  CHECK(f1.is_open()) << "Error opening file!";
  f1 << data_prob.format(csv_format);
  f1.close();

  LOG(INFO)<<"Map entropy after updation: "<<computeEntropy();

}


void GridMap::updateMapFromImage(
    const cv_bridge::CvImagePtr& cv_image_ptr,
    const geometry_msgs::Pose& mav_pose,
    const kindr::minimal::QuatTransformation& T_IMU_CAM,
    const double& sensor_fov_angle_x, const double& sensor_fov_angle_y,
    const cv::Mat& intrinsics_matrix, 
    const std::shared_ptr<voxblox::EsdfMap>& esdf_map_,
    const double width, const double height, const double min_z, const double max_z,
    const std::string sensor_type,
    std::unique_ptr<human_detection::HumanDetector>& human_detector_ptr,
    const double run_time, const double& saturation_height,
    const std::vector<double>& true_positive_coeffs,
    const std::vector<double>& false_negative_coeffs) 
{
  // If position is not inside the boundary, don't update anything
  if(!insideBoundary(width,height,min_z,max_z,mav_pose.position))
    return;

  // Obtain world to IMU (vicon-sensor-body) transform.
  kindr::minimal::QuatTransformation T_MAP_IMU;
  tf::poseMsgToKindr(mav_pose, &T_MAP_IMU);
  // Compute world to camera transform.
  kindr::minimal::QuatTransformation T_MAP_CAM = T_MAP_IMU * T_IMU_CAM;
  geometry_msgs::Pose camera_pose;
  tf::poseKindrToMsg(T_MAP_CAM, &camera_pose);

  // Read the image to project to the map.
  cv::Mat image = cv_image_ptr->image;

  // Extract the RGB color channels.
  cv::Mat channels[3];
  // Extract the LAB channels.
  // cv::cvtColor(image, image, cv::COLOR_BGR2Lab);


  split(image, channels);
  channels[0] = (unsigned int)(0*255)*cv::Mat::ones(cv::Size(image.cols,image.rows),CV_8UC1);

  std::vector<darknet_ros::RosBox_> rect_points;
  detectHuman(image,human_detector_ptr,rect_points,intrinsics_matrix,T_MAP_CAM);

    // Change the pixel values at human detection points in the image to the probability
  if(!rect_points.empty())
  {
    for (size_t i = 0; i < image.rows; ++i) 
    {
      for (size_t j = 0; j < image.cols; ++j) 
      {
        for (const darknet_ros::RosBox_& bound_rect : rect_points) 
        {
          // LOG(INFO)<<"Updating person in map "<<i<<","<<j;
          if(ifInsideRectangle(bound_rect,i,j))
          {
            // LOG(INFO)<<"About to copy probability as "<<(bound_rect.prob*255);
            channels[0].at<uchar>(i,j) = (unsigned int)(1*255); //(unsigned int)bound_rect.prob*255;
            break;
          }
        }
      }
    }
  }

  capture_id++;
  logger_.writeImage(image,capture_id);
  logger_.writeImage(image,1);
  logger_.writeImage(channels[0],0);

  // LOG(INFO)<<"Back to updatemapfroimimage";

  // Calculate the corner co-ordinates of the observed submap.
  SubmapCoordinates submap_coordinates;
  Eigen::Vector4d p1;
  p1 << 0.0, 0.0, 1.0, 1.0;
  Eigen::Vector4d p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.upper_left = p2.head(2);
  p1 << 0.0, (image.rows - 1.0), 1.0, 1.0;
  p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.lower_left = p2.head(2);
  p1 << (image.cols - 1.0), (image.rows - 1.0), 1.0, 1.0;
  p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.lower_right = p2.head(2);
  p1 << (image.cols - 1.0), 0.0, 1.0, 1.0;
  p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.upper_right = p2.head(2);
  environmentToGridCoordinates(&submap_coordinates);
  trimSubmapCoordinatesToGrid(&submap_coordinates);

  // Initialize variables to keep track of submap data.
  const size_t x_min = std::min(
      {submap_coordinates.lower_left(0), submap_coordinates.upper_left(0)});
  const size_t x_max = std::max(
      {submap_coordinates.upper_right(0), submap_coordinates.lower_right(0)});
  const size_t y_min = std::min(
      {submap_coordinates.lower_left(1), submap_coordinates.lower_right(1)});
  const size_t y_max = std::max(
      {submap_coordinates.upper_left(1), submap_coordinates.upper_right(1)});
  const size_t submap_size_x = x_max - x_min + 1;
  const size_t submap_size_y = y_max - y_min + 1;
  Eigen::MatrixXd submap_counter, submap_data;
  submap_counter.setZero(submap_size_y, submap_size_x);
  submap_data.setZero(submap_size_y, submap_size_x);

  for (size_t i = 0; i < image.rows; ++i) {
    for (size_t j = 0; j < image.cols; ++j) {
      // Apply the homography transform to each image pixel.
      Eigen::Vector4d image_point;
      image_point << j, i, 1.0, 1.0;
      Eigen::Vector4d transformed_point =
          projectPixelToGround(intrinsics_matrix, T_MAP_CAM, image_point);

      Eigen::Vector2d grid_point = transformed_point.head(2);


      // LOG(INFO)<<"("<<i<<","<<j<<"), grid point ("<<grid_point[0]<<","<<grid_point[1]<<")";
      //std::this_thread::sleep_for (std::chrono::milliseconds(1));
      geometry_msgs::Point ground_point;
      ground_point.x = grid_point[0];
      ground_point.y = grid_point[1];
      ground_point.z = 0.0;
      if(!onGroundPlane(width,height,ground_point))
        continue;
      if(!isGroundVisible(mav_pose.position,ground_point,esdf_map_))
        continue;


      environmentToGridCoordinates(&grid_point);
      // Skip measurement: out-of-bounds.
      if (grid_point(0) - x_min < 0 ||
          grid_point(0) - x_min > (submap_size_x - 1) ||
          grid_point(1) - y_min < 0 ||
          grid_point(1) - y_min > (submap_size_y - 1) || grid_point(0) < 0 ||
          grid_point(1) < 0 || grid_point(0) > (length_(0) - 1) ||
          grid_point(1) > (length_(1) - 1)) {
        continue;
      }
      submap_counter(grid_point(1) - y_min, grid_point(0) - x_min)++;
      submap_data(grid_point(1) - y_min, grid_point(0) - x_min) =
          (unsigned int)channels[0].at<uchar>(i, j);
      //std::cout<<",Hue: "<<(unsigned int)channels[0].at<uchar>(i, j);
    }
  }

  // Identify valid measuremsents with enough pixel mappings
  for (size_t i = 0; i < submap_data.rows(); ++i) {
    for (size_t j = 0; j < submap_data.cols(); ++j) {
      // Skip measurement: not enough pixels in cell.
      if (submap_counter(i, j) < 1) {
        continue;
      }

      // Occupancy probability given the current measurement.
      double p;
      // Occupied: true positive sensor model.
      if (submap_data(i, j) == (unsigned int)(1*255)) {
        LOG(INFO)<<"found human grid";
        double p_z = toProbability(data_(y_min + i, x_min + j));
        // p = computeTruePositiveProbability(camera_pose.position.z, saturation_height, true_positive_coeffs) * p_z +
        //     computeFalseNegativeProbability(camera_pose.position.z, saturation_height, false_negative_coeffs) * (1-p_z);

        p = computeTruePositiveProbability(
            camera_pose.position.z, saturation_height, true_positive_coeffs);
        // LOG(INFO)<<"updated data from "<<data_(y_min + i, x_min + j)<<" to "<<data_(y_min + i, x_min + j)+std::log(p / (1 - p)) ;
        // Free: false positive sensor model.
      } else {

        // double p_z = toProbability(data_(y_min + i, x_min + j));
        // p = computeFalseNegativeProbability(camera_pose.position.z, saturation_height, true_positive_coeffs) * p_z +
        //     computeTruePositiveProbability(camera_pose.position.z, saturation_height, false_negative_coeffs) * (1-p_z);

        p = computeFalseNegativeProbability(
            camera_pose.position.z, saturation_height, false_negative_coeffs);
      }

      data_(y_min + i, x_min + j) += std::log(p / (1 - p));// - std::log(map_prior / (1 - map_prior));
    }
  }

  // Convert data to probability and save
  Eigen::MatrixXd data_prob, entropy_dist;
  data_prob.resize(length_(0), length_(1));
  entropy_dist.resize(length_(0), length_(1));
  for (auto i = 0; i < data_.rows(); ++i) 
    for (auto j = 0; j < data_.cols(); ++j) 
    {
      data_prob(i,j) = 1.0 - (1.0 / (1.0 + std::exp(data_(i, j))));
      entropy_dist(i,j) = -(data_prob(i,j) * std::log(data_prob(i,j)) + (1.0 - data_prob(i,j)) * std::log(1.0 - data_prob(i,j)));
    }

  logger_.writeToCSV(data_prob,"data_");
  logger_.writeToCSV(entropy_dist,"entropy_");

  LOG(INFO)<<"Map entropy after image "<<capture_id<<" : "<<computeEntropy();
   
}

void GridMap::detectHuman(cv::Mat& image1, 
  std::unique_ptr<human_detection::HumanDetector>& human_detector_ptr, 
  std::vector<darknet_ros::RosBox_>& rect_points,
  const cv::Mat& intrinsics_matrix,
  const kindr::minimal::QuatTransformation& T_MAP_CAM)
{
  // LOG(INFO)<<"Starting human detector";
  
  // Converting to cvbridge type
  cv_bridge::CvImagePtr input_image_ptr(new cv_bridge::CvImage);
  input_image_ptr->image = image1;
  input_image_ptr->encoding = "bgr8";

  // Doing detection
  cv_bridge::CvImagePtr output_image_ptr;
  auto t_start = std::chrono::high_resolution_clock::now();
  LOG(INFO) << "About to detect";
  human_detector_ptr->detectBoxInImage(input_image_ptr, &rect_points, &output_image_ptr);

  // Get rid of obvious false positives (giants)
  removeGiants(rect_points,intrinsics_matrix, T_MAP_CAM, &output_image_ptr);
  auto t_end = std::chrono::high_resolution_clock::now();

  // Timing info
  LOG(INFO)
      << "Time for human detector: "
      << std::chrono::duration<double, std::milli>(t_end - t_start).count()
      << " ms\n";
  image1 = output_image_ptr->image;
}

void GridMap::removeGiants(
    std::vector<darknet_ros::RosBox_>& rect_points,
    const cv::Mat& intrinsics_matrix,
    const kindr::minimal::QuatTransformation& T_MAP_CAM,
    cv_bridge::CvImagePtr* output_image_ptr)//, cv_bridge::CvImagePtr* image_ptr)
{
    // CHECK_NOTNULL(image_ptr);
  int i = 0;
  std::deque<int> del_ind;
  del_ind.clear();
  for (const darknet_ros::RosBox_& bound_rect : rect_points) 
  {
    // top left point of rectangle
    int i1 = bound_rect.y-bound_rect.h/2;
    int j1 = bound_rect.x-bound_rect.w/2;
    Eigen::Vector2d point1;
    point1 << j1,i1;
    pixelToEnvironment(point1,intrinsics_matrix,T_MAP_CAM);
    // LOG(INFO)<<"Pixel: "<<j1<<","<<i1<<"   grid:"<<point1[0]<<","<<point1[1];

    // bottom left point of rectnagle
    int i2 = bound_rect.y+bound_rect.h/2;
    int j2 = bound_rect.x-bound_rect.w/2;      
    Eigen::Vector2d point2;
    point2 << j2,i2;
    pixelToEnvironment(point2,intrinsics_matrix,T_MAP_CAM);

    // top right point of rectnagle
    int i3 = bound_rect.y-bound_rect.h/2;
    int j3 = bound_rect.x+bound_rect.w/2;       
    Eigen::Vector2d point3;
    point3 << j3,i3;
    pixelToEnvironment(point3,intrinsics_matrix,T_MAP_CAM);   

    double rect_width = sqrt(pow(point1[0]-point3[0],2)+pow(point1[1]-point3[1],2));  
    double rect_height = sqrt(pow(point1[0]-point2[0],2)+pow(point1[1]-point2[1],2)); 
    LOG(INFO)<<"Rectangle width, height: "<<rect_width<<","<<rect_height;

    if(rect_width > 3 || rect_height >3)
    {
      del_ind.push_back(i);

      cv::Point pt1((bound_rect.x-bound_rect.w/2),\
       (bound_rect.y-bound_rect.h/2));
      cv::Point pt2((bound_rect.x+bound_rect.w/2),\
         (bound_rect.y+bound_rect.h/2));
      cv::rectangle((*output_image_ptr)->image, pt1,pt2,cv::Scalar(0, 0, 255), 5,5);
    }
    i++;
  }

  for (int i = 0;i<del_ind.size();i++) 
  {
    LOG(INFO)<<"Deleting giant number: "<<del_ind.at(i)+1;
    rect_points.erase(rect_points.begin()+del_ind.at(i)-i); // account for decreasing size of rect_points while in the loop
  }

}

void GridMap::pixelToEnvironment(
    Eigen::Vector2d& point,
    const cv::Mat& intrinsics_matrix,
    const kindr::minimal::QuatTransformation& T_MAP_CAM)
{
  // Apply the homography transform to the pixel.
  Eigen::Vector4d image_point;
  image_point << point[0], point[1], 1.0, 1.0;
  Eigen::Vector4d transformed_point =
      projectPixelToGround(intrinsics_matrix, T_MAP_CAM, image_point);
  point = transformed_point.head(2);
  // environmentToGridCoordinates(&point);
}


void GridMap::predictMapUpdate(
    const geometry_msgs::Pose& mav_pose,
    const kindr::minimal::QuatTransformation& T_IMU_CAM,
    const double& sensor_fov_angle_x, const double& sensor_fov_angle_y,
    const double& saturation_height,
    const std::vector<double>& true_positive_coeffs,
    const std::vector<double>& false_negative_coeffs,
    const std::shared_ptr<voxblox::EsdfMap>& esdf_map_,
    const double width, const double height, const double min_z, const double max_z) {

  // If the measurement location is outside the boundary don't update anything
  if(!insideBoundary(width,height,min_z,max_z,mav_pose.position))
    return; 

  // Obtain map to IMU (vicon-sensor-body) transform.
  kindr::minimal::QuatTransformation T_MAP_IMU;
  tf::poseMsgToKindr(mav_pose, &T_MAP_IMU);
  // Compute map to camera transform.
  kindr::minimal::QuatTransformation T_MAP_CAM = T_MAP_IMU * T_IMU_CAM;
  geometry_msgs::Pose camera_pose;
  tf::poseKindrToMsg(T_MAP_CAM, &camera_pose);

  // Calculate the image coverage [m] based on the camera position,
  // assuming camera is perfectly aligned with the map.
  const auto image_edge_size = getImageEdgeSize(
      camera_pose.position.z, sensor_fov_angle_x, sensor_fov_angle_y);

  // Find bounds of the image given the current position.
  const auto submap_coordinates =
      getRectSubmapCoordinates(camera_pose.position, image_edge_size);
  auto submap_coordinates_grid = submap_coordinates;
  environmentToGridCoordinates(&submap_coordinates_grid);
  trimSubmapCoordinatesToGrid(&submap_coordinates_grid);

  // Update all points inside observed submap assuming ML measurements.
  for (size_t i = submap_coordinates_grid.lower_left(0);
       i <= submap_coordinates_grid.upper_right(0); ++i) {
    for (size_t j = submap_coordinates_grid.lower_left(1);
         j <= submap_coordinates_grid.upper_right(1); ++j) {

      geometry_msgs::Point point;
      point.x = i;    // Checked, but do confirm with Masha
      point.y = j;
      point.z = 0.0;
      gridToEnvironmentCoordinates(&point);

      // If the point is not on the ground skip updation
      if(!onGroundPlane(width,height,point))
        continue;

      // if ground is not visible don't update the covariance
      if(!isGroundVisible(mav_pose.position,point,esdf_map_))
        continue;

      // Occupancy probability given the current measurement.
      double p;
      // Occupied: True positive sensor model.
      if (data_(j, i) > 0) {

        // double p_z = toProbability(data_(j, i));
        // p = computeTruePositiveProbability(camera_pose.position.z, saturation_height, true_positive_coeffs) * p_z +
        //     computeFalseNegativeProbability(camera_pose.position.z, saturation_height, false_negative_coeffs) * (1-p_z);

        p = computeTruePositiveProbability(
            camera_pose.position.z, saturation_height, true_positive_coeffs);
      }
      // Free: False positive sensor model.
      else {

        // double p_z = toProbability(data_(j, i));
        // p = computeFalseNegativeProbability(camera_pose.position.z, saturation_height, true_positive_coeffs) * p_z +
        //     computeTruePositiveProbability(camera_pose.position.z, saturation_height, false_negative_coeffs) * (1-p_z);

        p = computeFalseNegativeProbability(
            camera_pose.position.z, saturation_height, false_negative_coeffs);
      }

      // double p_z = toProbability(data_(j, i));
      // p = computeTruePositiveProbability(camera_pose.position.z, saturation_height, true_positive_coeffs) * p_z +
      //     computeFalseNegativeProbability(camera_pose.position.z, saturation_height, false_negative_coeffs) * (1-p_z);

      data_(j, i) += std::log(p / (1 - p));// - std::log(map_prior / (1 - map_prior));
      // LOG(INFO)<<"data "<<data_(j,i);
    }
  }
}

bool GridMap::onGroundPlane(const double width, const double height, const geometry_msgs::Point& ground_point)
{
  if(ground_point.x<height/2 && ground_point.x>-height/2 &&
     ground_point.y<width/2 && ground_point.y>-width/2 &&
     ground_point.z < 0.0001 && ground_point.z >-0.0001)
    return true;
  else
    return false;
}

bool GridMap::insideBoundary(const double width, const double height,
  const double min_z, const double max_z, const geometry_msgs::Point& mav_position)
{
  if(mav_position.x<height/2 && mav_position.x>-height/2 && mav_position.y<width/2 && mav_position.y>-width/2 
     && mav_position.z<max_z && mav_position.z>min_z)
    return true;
  else
    return false;
}

bool GridMap::isGroundVisible(const geometry_msgs::Point& mav_position,
  const geometry_msgs::Point& ground_point, const std::shared_ptr<voxblox::EsdfMap>& esdf_map_)
{
  // LOG(INFO)<<"ground_point ("<<ground_point.x<<","<<ground_point.y<<\
  //       ") mav_pose ("<<mav_position.x<<","<<mav_position.y<<","<<mav_position.z<<")";
  //     std::this_thread::sleep_for (std::chrono::milliseconds(1));


  double norm_dist, dist_incr = 0.05, dist_curr;

  // Check if UAV is close to obstacles/boundary already
  Eigen::Vector3d mav_position_copy; // to deal with data type bullshit
  mav_position_copy[0] = mav_position.x;
  mav_position_copy[1] = mav_position.y;
  mav_position_copy[2] = mav_position.z; 
  if(esdf_map_->getDistanceAtPosition(mav_position_copy,&dist_curr))
    {
      if(dist_curr<=0.5)
        dist_incr = 0.5;    // Throw the first point out of the obstacle
    }
  else
    LOG(ERROR)<<"The current UAV position is outside the map! Inside isGroundVisible() ";

  norm_dist = getDistanceBetweenPoints(mav_position,ground_point);
  double u = dist_incr/norm_dist;

  Eigen::Vector3d in_point;
  //geometry_msgs::Point in_point_dup;  // to deal with data type bullshit
  while(u<1)
  {
    in_point[0] = (1-u)*mav_position.x + u*ground_point.x;
    in_point[1] = (1-u)*mav_position.y + u*ground_point.y;
    in_point[2] = (1-u)*mav_position.z + u*ground_point.z;

    // in_point_dup.x = in_point[0];
    // in_point_dup.y = in_point[1];
    // in_point_dup.z = in_point[2];

    double min_dist;
    if(esdf_map_->getDistanceAtPosition(in_point,&min_dist))
    {      
      if(min_dist<=0.2) 
      {
        double dist_to_ground_point = in_point[2];//getDistanceBetweenPoints(in_point_dup,ground_point);
        if(dist_to_ground_point > 0.2)
          return false; 
        else 
          return true;
      }
      
    }
    else
    {
      LOG(ERROR)<<"The point ("<<in_point[0]<<","<<in_point[1]<<","<<in_point[2] \
        <<") doesn't exist in the map or is not observed! Inside isGroundVisible()";
        return false;
    }
    u = u + (min_dist/norm_dist);
  }
}

Eigen::Vector4d GridMap::projectPixelToGround(
    const cv::Mat& intrinsics_matrix,
    const kindr::minimal::QuatTransformation& T_MAP_CAM,
    const Eigen::Vector4d& point) {
  // Full-rank camera extrinsics matrix for the homography.
  Eigen::Matrix4d E = T_MAP_CAM.getTransformationMatrix();
  // Full-rank camera calibration intrinsics matrix for the homography.
  Eigen::Matrix4d K;
  K << intrinsics_matrix.at<double>(0, 0), 0,
      intrinsics_matrix.at<double>(0, 2), 0, 0,
      intrinsics_matrix.at<double>(1, 1), intrinsics_matrix.at<double>(1, 2), 0,
      0, 0, 1, 0, 0, 0, 0, 1;
  // Full-rank camera matrix (intrinsics + extrinsics).
  Eigen::Matrix4d P = K * E.inverse();

  // Apply the homography transform to find the ray projected from the camera.
  Eigen::Vector4d camera_point = E.block(0, 3, 1, 4);
  Eigen::Vector4d ray_vector = (P.inverse() * point) - camera_point;
  // Find the intersection of the image ray and the ground plane.
  return camera_point - (camera_point(2) / ray_vector(2)) * ray_vector;
}

void GridMap::toOccupancyGrid(nav_msgs::OccupancyGrid* occupancy_grid) {
  CHECK_NOTNULL(occupancy_grid);
  occupancy_grid->header.frame_id = frame_id_;
  occupancy_grid->header.stamp = ros::Time::now();
  occupancy_grid->info.map_load_time = occupancy_grid->header.stamp;
  // Map dimensions [cells].
  occupancy_grid->info.width = length_(0);
  occupancy_grid->info.height = length_(1);
  // Assume same resolution in both grid dimensions.
  occupancy_grid->info.resolution = resolution_(0);
  // Map position [m].
  occupancy_grid->info.origin.position.x = position_(0);
  occupancy_grid->info.origin.position.y = position_(1);
  occupancy_grid->info.origin.position.z = 0.0;
  occupancy_grid->info.origin.orientation.x = 0.0;
  occupancy_grid->info.origin.orientation.y = 0.0;
  occupancy_grid->info.origin.orientation.z = 0.0;
  occupancy_grid->info.origin.orientation.w = 1.0;
  occupancy_grid->data.resize(length_(0) * length_(1));
  // Row-major ordering.
  for (auto i = 0; i < data_.rows(); ++i) {
    for (auto j = 0; j < data_.cols(); ++j) {
      occupancy_grid->data[(j + i * data_.cols())] =
          toProbability(data_(i, j)) * 100;
    }
  }
}

double GridMap::calculateRMSE()
{
  // Convert data to probability
  Eigen::MatrixXd data_prob;
  data_prob.resize(length_(0), length_(1));

  for (auto i = 0; i < data_.rows(); ++i) 
    for (auto j = 0; j < data_.cols(); ++j) 
      data_prob(i,j) = 1.0 - (1.0 / (1.0 + std::exp(data_(i, j))));

  Eigen::MatrixXd diff = data_prob - ground_truth;
  diff = diff.cwiseProduct(diff);
  // LOG(INFO)<<"Size of diff: "<<diff.rows()<<" , "<<diff.cols();

  logger_.writeToCSV(diff,"weight");

  return sqrt(diff.sum());
  // return sqrt(diff.sum()/(data_.rows()*data_.cols()));
}

double GridMap::calculateWRMSE()
{
  // Convert data to probability
  Eigen::MatrixXd data_prob;
  data_prob.resize(length_(0), length_(1));

  for (auto i = 0; i < data_.rows(); ++i) 
    for (auto j = 0; j < data_.cols(); ++j) 
      data_prob(i,j) = 1.0 - (1.0 / (1.0 + std::exp(data_(i, j))));
    
  // Eigen::MatrixXd aa1 = (data_.array() > 0.3).cast<double>();

  // Eigen::MatrixXd aa1 = data_;
  // for (size_t j = 0; j < data_.cols(); ++j) 
  //   for (size_t i = 0; i < data_.rows(); ++i) 
  //     if(data_(i,j)>0.5)
  //       aa1(i,j) = 1;

  Eigen::MatrixXd diff = data_prob - ground_truth;
  diff = diff.cwiseAbs();

  Eigen::MatrixXd weight = ground_truth;
  // double w_sum = weight.sum();
  // weight = weight/w_sum;
  // weight = (weight-(weight.minCoeff()*Eigen::MatrixXd::Ones(data_.rows(),data_.cols())))/(weight.maxCoeff()-weight.minCoeff());

  // LOG(INFO)<<"Weight 10,10: "<<weight(10,10)<<"sum: "<<w_sum;
  LOG(INFO)<<"Size of weight: "<<weight.rows()<<" , "<<weight.cols();

  diff = diff.cwiseProduct(weight);
  logger_.writeToCSV(diff,"weight");

  return sqrt(diff.sum());
  // return sqrt(diff.sum()/(data_.rows()*data_.cols()));
}

double GridMap::computeTruePositiveProbability(
    const double& camera_height, const double& saturation_height,
    const std::vector<double>& true_positive_coeffs) const {
  if (camera_height > saturation_height) {
    return 0.5;
  } else {
    return 0.95*std::exp(-0.5*std::pow((camera_height-15)/20,2));
    // return true_positive_coeffs[0] * pow(camera_height, 2) +
    //        true_positive_coeffs[1] * camera_height + true_positive_coeffs[2];
  }
}

double GridMap::computeFalseNegativeProbability(
    const double& camera_height, const double& saturation_height,
    const std::vector<double>& false_negative_coeffs) const {
  if (camera_height > saturation_height) {
    return 0.5;
  } else {
    return 1- (0.8*std::exp(-0.5*std::pow((camera_height-15)/20,2)));
    // return false_negative_coeffs[0] * pow(camera_height, 2) +
    //        false_negative_coeffs[1] * camera_height + false_negative_coeffs[2];
  }
}

void GridMap::gridToEnvironmentCoordinates(geometry_msgs::Point* point) const {
  // Add half a cell size in the conversion to get the center of the cell.
  point->x = point->x * resolution_(0) + position_(0) + resolution_(0) / 2.0;
  point->y = point->y * resolution_(1) + position_(1) + resolution_(1) / 2.0;
}

void GridMap::environmentToGridCoordinates(geometry_msgs::Point* point) const {
  // Round down here, because the points here are indices to the grid data
  // structure (starting from 0).
  double f1;
  std::modf((point->x - position_(0)) / resolution_(0), &f1);
  point->x = f1;
  std::modf((point->y - position_(1)) / resolution_(1), &f1);
  point->y = f1;
}

void GridMap::environmentToGridCoordinates(Eigen::Vector2d* point) const {
  double f1;
  std::modf(((*point)(0) - position_(0)) / resolution_(0), &f1);
  (*point)(0) = f1;
  std::modf(((*point)(1) - position_(1)) / resolution_(1), &f1);
  (*point)(1) = f1;
}

void GridMap::environmentToGridCoordinates(
    SubmapCoordinates* submap_coordinates) const {
  environmentToGridCoordinates(&(submap_coordinates->lower_left));
  environmentToGridCoordinates(&(submap_coordinates->upper_right));
  environmentToGridCoordinates(&(submap_coordinates->upper_left));
  environmentToGridCoordinates(&(submap_coordinates->lower_right));
}

void GridMap::trimSubmapCoordinatesToGrid(
    SubmapCoordinates* submap_coordinates) const {
  submap_coordinates->lower_left(0) =
      std::max(submap_coordinates->lower_left(0), 0.0);
  submap_coordinates->lower_left(1) =
      std::max(submap_coordinates->lower_left(1), 0.0);
  submap_coordinates->upper_right(0) =
      std::min(submap_coordinates->upper_right(0), length_(0) - 1.0);
  submap_coordinates->upper_right(1) =
      std::min(submap_coordinates->upper_right(1), length_(1) - 1.0);
  submap_coordinates->upper_left(0) =
      std::max(submap_coordinates->upper_left(0), 0.0);
  submap_coordinates->upper_left(1) =
      std::min(submap_coordinates->upper_left(1), length_(1) - 1.0);
  submap_coordinates->lower_right(0) =
      std::min(submap_coordinates->lower_right(0), length_(0) - 1.0);
  submap_coordinates->lower_right(1) =
      std::max(submap_coordinates->lower_right(1), 0.0);
}

double GridMap::toProbability(double x) const {
  // Set unknown occupancy (-1).
  if (x == 0)
    return -1.0 / 100;
  else
    return 1.0 - (1.0 / (1.0 + std::exp(x)));
}

double GridMap::toLogOdds(double x) const {
  // Set unknown occupancy.
  if (x == -1.0 / 100)
    return 0;
  else
    return std::log(x / (1.0 - x));
}

bool GridMap::ifInsideRectangle(const darknet_ros::RosBox_& rect_point, const size_t i,const size_t j)
{
  // LOG(INFO)<<"Entering ifInsideRectangle";
  if(j>=rect_point.x-rect_point.w/2 && j<=rect_point.x+rect_point.w/2 && i>=rect_point.y-rect_point.h/2 && i<=rect_point.y+rect_point.h/2)
    return true;
  else
    return false;
}

double GridMap::getDistanceBetweenPoints(const geometry_msgs::Point& p1,
                                             const geometry_msgs::Point& p2) {
  return sqrt(pow(p1.x - p2.x, 2.0) + pow(p1.y - p2.y, 2.0) +
              pow(p1.z - p2.z, 2.0));
}