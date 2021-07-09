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

#include "tmplanner_continuous/grid_map.h"

using namespace grid_map;

GridMap::GridMap(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
      nh_(nh),
      nh_private_(nh_private) {};

void GridMap::setMapGeometry(const double& width, const double& height,
                             const double& resolution_x,
                             const double& resolution_y,
                             const double& position_x, const double& position_y,
                             const std::string& frame_id,
                             const std::string sensor_type) {
  CHECK_GT(resolution_x, 0) << "Error in map resolution value in x.";
  CHECK_GT(resolution_y, 0) << "Error in map resolution value in y.";
  CHECK_GE(width, 0) << "Error in map width value.";
  CHECK_GE(height, 0) << "Error in map height value.";

  frame_id_ = frame_id;
  resolution_(0) = resolution_x;
  resolution_(1) = resolution_y;
  length_(0) = static_cast<int>(round(height / resolution_x));
  length_(1) = static_cast<int>(round(width / resolution_y));

  data_.resize(length_(0), length_(1));
  acquisition.resize(length_(0), length_(1));
  acquisition_view = 0;

  // Center the map corners.
  position_(0) = position_x;
  position_(1) = position_y;

  capture_id = 0;

  setGroundTruth(sensor_type,width,height);
  logger_.writeGroundTruth(ground_truth);
}

void GridMap::setGroundTruth(const std::string sensor_type, const double& width, const double& height)
{
  if(sensor_type == "human")
  {
    ground_truth.resize(length_(0), length_(1));
    ground_truth.setZero(length_(0), length_(1));
    // ground_truth.setOnes(length_(0), length_(1));
    // ground_truth = ground_truth*0.1;

    //////  Humans on bottom side ///////////
    // Eigen::Vector2d human_loc;
    // human_loc[0] = 8;    human_loc[1] = -12;
    // addHumanToGroundTruth(human_loc,width,height);
    // human_loc[0] = -6;    human_loc[1] = -7;
    // addHumanToGroundTruth(human_loc,width,height);
    // human_loc[0] = 12;    human_loc[1] = -7;
    // addHumanToGroundTruth(human_loc,width,height);      
    // human_loc[0] = -7;    human_loc[1] = -12;
    // addHumanToGroundTruth(human_loc,width,height);
    // human_loc[0] = 4;    human_loc[1] = -11;
    // addHumanToGroundTruth(human_loc,width,height);
    // human_loc[0] = -12;    human_loc[1] = -6;
    // addHumanToGroundTruth(human_loc,width,height);
    // human_loc[0] = 0;    human_loc[1] = -7;
    // addHumanToGroundTruth(human_loc,width,height); 
    //////  Humans on bottom side ///////////

    //////  Final real life scene ///////////
    // Eigen::Vector2d human_loc;
    // human_loc[0] = 8;    human_loc[1] = -2;
    // addHumanToGroundTruth(human_loc,width,height);
    // human_loc[0] = 10;    human_loc[1] = 3;
    // addHumanToGroundTruth(human_loc,width,height);     
    // human_loc[0] = 13;    human_loc[1] = -2;
    // addHumanToGroundTruth(human_loc,width,height);
    // human_loc[0] = 7;    human_loc[1] = 2;
    // addHumanToGroundTruth(human_loc,width,height);
    //////  Final real life scene ///////////

        //////  ICRA paper humans ///////////
    Eigen::Vector2d human_loc;
    human_loc[0] = -10;    human_loc[1] = -10;
    addHumanToGroundTruth(human_loc,width,height);
    human_loc[0] = -10;    human_loc[1] = 8;
    addHumanToGroundTruth(human_loc,width,height);     
    human_loc[0] = 12;    human_loc[1] = -5;
    addHumanToGroundTruth(human_loc,width,height);
    human_loc[0] = 0;    human_loc[1] = 8;
    addHumanToGroundTruth(human_loc,width,height);
    human_loc[0] = 0;    human_loc[1] = -12;
    addHumanToGroundTruth(human_loc,width,height);     
    human_loc[0] = 10;    human_loc[1] = 2;
    addHumanToGroundTruth(human_loc,width,height);
    human_loc[0] = -8;    human_loc[1] = -8;
    addHumanToGroundTruth(human_loc,width,height);
    //////  ICRA paper humans ///////////
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

void GridMap::fillUnknown() {
  data_.setOnes(length_(0), length_(1));
  data_ = data_ * 0.1;
}

void GridMap::fillMapData(const Eigen::MatrixXd& data) {
  CHECK_EQ(length_(0), data.rows())
      << "Data dimensions don't match map dimensions in x.";
  CHECK_EQ(length_(1), data.cols())
      << "Data dimensions don't match map dimensions in y.";
  data_ = data;
}

void GridMap::computeCovariance(
    const std::string cov_fun, const std::vector<double> log_hyperparams_vector,
    const double& resolution_x_predict, const double& resolution_y_predict,
    const std::shared_ptr<voxblox::EsdfMap>& esdf_map_,
    const std::string cmaes_objective) {
  if (cov_fun == "matern3") {
    gp::CovMatern3iso<2, double> cov;
    Eigen::Vector2d log_hyperparams;
    log_hyperparams << log_hyperparams_vector[0],
        log_hyperparams_vector[1];  // Length scale, signal variance squared
    cov.setLogHyperparams(log_hyperparams);

    // Create the inputs.
    // Mapping resolution.
    const unsigned int num_inputs = length_(0) * length_(1);
    gp::CovMatern3iso<2, double>::Inputs X(2, num_inputs);
    size_t counter = 0;
    for (size_t i = 1; i <= length_(0); ++i) {
      for (size_t j = 1; j <= length_(1); ++j) {
        X(0, counter) = i;
        X(1, counter) = j;
        counter++;
      }
    }

    // Predictive resolution.
    const size_t length_x_predict =
        length_(0) * (resolution_(0) / resolution_x_predict);
    const size_t length_y_predict =
        length_(1) * (resolution_(1) / resolution_y_predict);

    const size_t num_inputs_predict = length_x_predict * length_y_predict;
    gp::CovMatern3iso<2, double>::Inputs X_predict(2, num_inputs_predict);
    counter = 0;
    for (size_t i = 1; i <= length_x_predict; ++i) {
      for (size_t j = 1; j <= length_y_predict; ++j) {
        X_predict(0, counter) = i;
        X_predict(1, counter) = j;
        counter++;
      }
    }

    // Compute the covariance using the kernel.
    Eigen::MatrixXd K, Kss, Kst;
    K.resize(num_inputs, num_inputs);
    Kss.resize(num_inputs_predict, num_inputs_predict);
    Kst.resize(num_inputs_predict, num_inputs);

    cov.getCovarianceMatrix(X, &K);
    // Add noise variance hyperparameter from likelihood function.
    K = K + Eigen::MatrixXd::Identity(K.rows(), K.cols()) * exp(2 * 0.35);
    cov.getCovarianceMatrix(X_predict, &Kss);
    cov.getCovarianceMatrix(X_predict, X, &Kst);

    covariance_ = Kss - Kst * K.inverse() * Kst.transpose();
    addObstaclesGP(esdf_map_,cmaes_objective);
  }
}

void GridMap::addObstaclesGP(const std::shared_ptr<voxblox::EsdfMap>& esdf_map_, const std::string cmaes_objective)
{
  Eigen::VectorXd measurements, measurement_indices;
  measurements.setZero(length_(0)*length_(1)/4);
  measurement_indices.setZero(length_(0)*length_(1)/4);
  size_t counter = 0;

  LOG(INFO)<<"Grid length: "<<length_(0)<<","<<length_(1);
  LOG(INFO)<<"Covariance size: "<<covariance_.rows()<<","<<covariance_.cols();
  for(int i=0;i<length_(0);i++)
  {
    for(int j=0;j<length_(1);j++)
    {
      geometry_msgs::Point grid_ij;
      grid_ij.x = i;
      grid_ij.y = j;
      grid_ij.z = 0.5;  // obstacles in the horizontal plane above gound cannot be mapped
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
            // covariance_(i*length_(1)+(j),i*length_(1)+(j)) = 0.05; // set obstacle covariance to zero

            measurements(counter) = 0.03;
            measurement_indices(counter) = i*length_(1)+(j);
            counter++;
          }
        }
      else
        LOG(ERROR)<<"Point ("<<grid_vec[0]<<","<<grid_vec[1]<<","<<grid_vec[2]<<") not found in the map! Inside addObstacleGP() ";
    }
  }

  // Perform KF update so that obstacles are added to the GP model
  const double var = 0.06;
  const Eigen::SparseMatrix<double> H =
      constructMeasurementModel(measurement_indices.head(counter));
  KFUpdate(measurements.head(counter), var, H, cmaes_objective);

  // double pval = stats::pnorm(-1.32,5.32,11.2);
  // LOG(INFO)<<"CDF is : "<<pval;

  // boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::variance> > acc;
  // acc(7); acc(5); acc(16); acc(10);
  // //std::for_each(a_vec.begin(), a_vec.end(), std::bind<void>(std::ref(acc), _1));
  // LOG(INFO) << boost::accumulators::mean(acc);
  // LOG(INFO) << sqrt(boost::accumulators::variance(acc));

  // LOG(INFO) << "Maximum data "<< data_.maxCoeff();

  // if(optimization_parameters_.cmaes_objective == "acquisition_EI")
  //   expectedImprovement();
  // else if(optimization_parameters_.cmaes_objective == "acquisition_PI")
  //   probabilityOfImprovement();

  logger_.writeGPImages(data_,covariance_,acquisition);

}

double GridMap::computeCovarianceTrace() const { return covariance_.trace(); }

double GridMap::computeCovarianceTrace(const double& lower_threshold) const {
  double cov_trace = 0.0;
  Eigen::MatrixXd P_diag = covariance_.diagonal();
  Eigen::Map<const Eigen::MatrixXd> P(P_diag.data(), data_.rows(),
                                      data_.cols());
  for (size_t j = 0; j < data_.cols(); ++j) {
    for (size_t i = 0; i < data_.rows(); ++i) {
      if (data_(i, j) > lower_threshold) {
        cov_trace += P(i, j);
      }
    }
  }

  return cov_trace;
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

void GridMap::updateMapFromImage(
    const cv_bridge::CvImagePtr& cv_image_ptr,
    const geometry_msgs::Pose& mav_pose,
    const kindr::minimal::QuatTransformation& T_IMU_CAM,
    const double& sensor_fov_angle_x, const double& sensor_fov_angle_y,
    const cv::Mat& intrinsics_matrix, const double& sensor_coefficient_A,
    const double& sensor_coefficient_B,
    const std::shared_ptr<voxblox::EsdfMap>& esdf_map_,
    const double width, const double height, const double min_z, const double max_z,
    const std::string cmaes_objective, const std::string sensor_type,
    std::unique_ptr<human_detection::HumanDetector>& human_detector_ptr,
    const double run_time) 
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

  if(sensor_type=="color")
  {
    // Reduce resolution of image linearly with height
    double scale_image = ((mav_pose.position.z)*2); // 26*4 times
    cv::Mat image1;
    cv::resize(image,image1,cv::Size(),1/scale_image,1/scale_image);
    cv::resize(image1,image,image.size(),0,0);

    // Extract the HSV channels.
    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
    split(image, channels);
  }
  else if(sensor_type=="human")
  {
    split(image, channels);
    channels[0] = (unsigned int)(0.05*255)*cv::Mat::ones(cv::Size(image.cols,image.rows),CV_8UC1);

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

              // Eigen::Vector2d point1;
              // point1 << j,i;
              // pixelToEnvironment(point1,intrinsics_matrix,T_MAP_CAM);
              // environmentToGridCoordinates(&point1);
              // LOG(INFO)<<"Checking ground truth: "<<ground_truth(point1[1],point1[0]);
              break;
            }
          }
        }
      }
    }
  
    // Extract the HSV channels.
    // cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
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
      submap_data(grid_point(1) - y_min, grid_point(0) - x_min) +=
          (unsigned int)channels[0].at<uchar>(i, j);
      //std::cout<<",Hue: "<<(unsigned int)channels[0].at<uchar>(i, j);
    }
  }

  // Identify valid measuremsents and their indices.
  Eigen::VectorXd measurements, measurement_indices;
  measurements.setZero(submap_size_x * submap_size_y);
  measurement_indices.setZero(submap_size_x * submap_size_y);
  size_t counter = 0;
  int true_positive = 0, true_negative = 0, false_positive = 0, false_negative = 0;
  for (size_t i = 0; i < submap_data.rows(); ++i) {
    for (size_t j = 0; j < submap_data.cols(); ++j) {
      // Skip measurement: not enough pixels in cell.
      if (submap_counter(i, j) < 100) {
        continue;
      }
      int row_coordinate = y_min + i;
      int col_coordinate = x_min + j;
      int index = row_coordinate + data_.rows() * col_coordinate;

      // Scale measurement to 0-1.
      measurements(counter) =
          (submap_data(i, j) / submap_counter(i, j)) / 255.0;
      measurement_indices(counter) = index;


      if(sensor_type=="human")
      {
        int truth = ground_truth(row_coordinate,col_coordinate);
        int predict =  (measurements(counter)>0.1);
        // LOG(INFO)<<"Ground truth at ("<<row_coordinate<<","<<col_coordinate<<") : "<<truth<<", update: "<<predict;
        if(truth == 1 && predict ==1)
          true_positive++;
        else if(truth == 0 && predict ==0)
          true_negative++;
        else if(truth == 1 && predict ==0)
          false_negative++;
        else if(truth == 0 && predict ==1)
          false_positive++;
      }


      counter++;
    }
  }

  // No valid measurements in this image.
  if (counter == 0) {
    return;
  }
  // Compute matrices for KF update.
  const double var = computeSensorNoiseVariance(
      camera_pose.position.z, sensor_coefficient_A, sensor_coefficient_B);
  const Eigen::SparseMatrix<double> H =
      constructMeasurementModel(measurement_indices.head(counter));
  KFUpdate(measurements.head(counter), var, H, cmaes_objective);
 
  LOG(INFO)<<"Covariance trace after image "<<capture_id<<" : "<<computeCovarianceTrace();
  if(cmaes_objective == "acquisition")
    LOG(INFO)<<"Acquisition sum after image "<<capture_id<<" : "<<getAcquisitionSum();

  //LOG(INFO)<<"Detections after image "<<capture_id<<" : "<<computeDetections();
  logger_.writeGPImages(data_,covariance_,acquisition);

  if(sensor_type=="human")
  {
    double recall = (double)true_positive/(double)(true_positive+false_negative);
    double precision = (double)true_positive/(double)(true_positive+false_positive);
    double F1_score = 2*precision*recall/(precision+recall);
    LOG(INFO)<<"recall: "<<recall<<", precision: "<<precision<<", F1 score: "<<F1_score;
    double RMSE = calculateRMSE();
    // LOG(INFO)<<"RMSE: "<<RMSE;
    logger_.writeClassifierMetrics(mav_pose.position.z,recall,precision,F1_score,RMSE);
  }

  
  // cv::imwrite(ros::package::getPath("tmplanner_continuous") +"/debug/matrices/variance_"+std::to_string(capture_id)+".png",a);
  // cv::imwrite(ros::package::getPath("tmplanner_continuous") +"/debug/matrices/data_"+std::to_string(capture_id)+".png",b);    
}

double GridMap::calculateRMSE()
{
  Eigen::MatrixXd diff = data_ - ground_truth;
  diff = diff.cwiseProduct(diff);
  // LOG(INFO)<<"Size of diff: "<<diff.rows()<<" , "<<diff.cols();

  std::ofstream f1;
  f1.open(ros::package::getPath("tmplanner_continuous") + "/debug/weight.csv");
  CHECK(f1.is_open()) << "Error opening file!";
  f1 << diff.format(csv_format);
  f1.close();

  return sqrt(diff.sum());
  // return sqrt(diff.sum()/(data_.rows()*data_.cols()));
}

double GridMap::calculateWRMSE()
{
  // Eigen::MatrixXd aa1 = (data_.array() > 0.3).cast<double>();

  // Eigen::MatrixXd aa1 = data_;
  // for (size_t j = 0; j < data_.cols(); ++j) 
  //   for (size_t i = 0; i < data_.rows(); ++i) 
  //     if(data_(i,j)>0.5)
  //       aa1(i,j) = 1;

  Eigen::MatrixXd diff = data_ - ground_truth;
  diff = diff.cwiseAbs();


  Eigen::MatrixXd P_diag = covariance_.diagonal();
  Eigen::Map<const Eigen::MatrixXd> variance(P_diag.data(), data_.rows(),
                                    data_.cols());

  // Eigen::MatrixXd weight = data_.cwiseProduct(variance);
  Eigen::MatrixXd weight = ground_truth;
  // double w_sum = weight.sum();
  // weight = weight/w_sum;
  // weight = (weight-(weight.minCoeff()*Eigen::MatrixXd::Ones(data_.rows(),data_.cols())))/(weight.maxCoeff()-weight.minCoeff());

  // LOG(INFO)<<"Weight 10,10: "<<weight(10,10)<<"sum: "<<w_sum;
  LOG(INFO)<<"Size of weight: "<<weight.rows()<<" , "<<weight.cols();

  diff = diff.cwiseProduct(weight);

  std::ofstream f1;
  f1.open(ros::package::getPath("tmplanner_continuous") + "/debug/weight.csv");
  CHECK(f1.is_open()) << "Error opening file!";
  f1 << diff.format(csv_format);
  f1.close();  


  return sqrt(diff.sum());
  // return sqrt(diff.sum()/(data_.rows()*data_.cols()));
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
  // LOG(INFO) << "About to detect";
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

bool GridMap::ifInsideRectangle(const darknet_ros::RosBox_& rect_point, const size_t i,const size_t j)
{
  // LOG(INFO)<<"Entering ifInsideRectangle";
  if(j>=rect_point.x-rect_point.w/2 && j<=rect_point.x+rect_point.w/2 && i>=rect_point.y-rect_point.h/2 && i<=rect_point.y+rect_point.h/2)
    return true;
  else
    return false;

}

double GridMap::computeDetections()  //replace this with RMSE or sort of measure
{ 
  double detections = 0.0;
  for (size_t j = 0; j < data_.cols(); ++j) {
    for (size_t i = 0; i < data_.rows(); ++i) {
        if(data_(i,j)>0.35)
          detections += data_(i,j);
    }
  }
  return detections;
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

void GridMap::predictMapUpdate(
    const geometry_msgs::Pose& mav_pose,
    const kindr::minimal::QuatTransformation& T_IMU_CAM,
    const double& sensor_fov_angle_x, const double& sensor_fov_angle_y,
    const double& sensor_coefficient_A, const double& sensor_coefficient_B,
    const std::shared_ptr<voxblox::EsdfMap>& esdf_map_,
    const double width, const double height, const double min_z, const double max_z,
    const std::string cmaes_objective, const bool if_kfupdate) {

  // LOG(INFO)<<"Start predictMapUpdate()";
  // If the measurement location is outside the boundary don't update anything
  if(!insideBoundary(width,height,min_z,max_z,mav_pose.position))
    return;


  // Obtain world to IMU (vicon-sensor-body) transform.
  kindr::minimal::QuatTransformation T_MAP_IMU;
  tf::poseMsgToKindr(mav_pose, &T_MAP_IMU);
  // Compute world to camera transform.
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
  const size_t submap_size_x = submap_coordinates_grid.upper_right(0) -
                               submap_coordinates_grid.lower_left(0) + 1;
  const size_t submap_size_y = submap_coordinates_grid.upper_right(1) -
                               submap_coordinates_grid.lower_left(1) + 1;

  // Get indices of the observed part of the environment (predicted submap).
  Eigen::VectorXd measurement_indices(submap_size_x * submap_size_y);
  acquisition_view = 0;
  size_t counter = 0;

  for (size_t i = submap_coordinates_grid.lower_left(0);
       i <= submap_coordinates_grid.upper_right(0); ++i) {
    for (size_t j = submap_coordinates_grid.lower_left(1);
         j <= submap_coordinates_grid.upper_right(1); ++j) {
      int row_coordinate = submap_coordinates_grid.upper_right(1) - j +
                           submap_coordinates_grid.lower_left(1);
      int col_coordinate = i;
      int index = row_coordinate + data_.rows() * col_coordinate;

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

      measurement_indices(counter) = index;
      counter++;

      if(cmaes_objective == "acquisition_EI" || cmaes_objective == "acquisition_PI" || cmaes_objective == "acquisition_UCB") 
      {
        //LOG(INFO)<<"(x,y) = ("<<i<<","<<j<<"), acq = "<<acquisition(i,j);      //check for what i and j is
        acquisition_view = acquisition_view+acquisition(j,i);
      }
    }
  }
 
   // If no useful observations are available don't update
  if(counter==0)
    return; 

  // copy to get to the right size
  Eigen::VectorXd measurement_indices_copy(counter);
  for(int i=0;i<counter;i++)
    measurement_indices_copy[i] = measurement_indices[i];
  
  // Compute matrices for KF update.
  const double var = computeSensorNoiseVariance(
      camera_pose.position.z, sensor_coefficient_A, sensor_coefficient_B);
  const Eigen::SparseMatrix<double> H =
      constructMeasurementModel(measurement_indices_copy);

  // Perform update for covariance only.
  if(cmaes_objective == "covariance")  
    KFUpdate(var, H,cmaes_objective);
  else if(cmaes_objective == "acquisition_EI" || cmaes_objective == "acquisition_PI" || cmaes_objective == "acquisition_UCB") 
  {
    // acquisition_view = 0.01*acquisition_view/(var*var);
    // acquisition_view = 0.1*acquisition_view/(var);  // Height dependent aquisition function model
    // acquisition_view = acquisition_view/(mav_pose.position.z); 
    acquisition_view = acquisition_view*(1/(7*std::sqrt(2*M_PI)))*std::exp(-0.5*std::pow((mav_pose.position.z-18)/7,2));
    if(if_kfupdate)   //update the GP only if map prediction is to be estimated sequentially
      KFUpdate(var, H,cmaes_objective);
  }

  //LOG(INFO)<<"Acq_view at ("<<mav_pose.position.x<<","<<mav_pose.position.y<<","<<mav_pose.position.z<<") = "<<acquisition_view;
  //LOG(INFO)<<"Debugging Acquisition sum: "<<getAcquisitionSum();

  // LOG(INFO)<<"End predictMapUpdate()";
}

void GridMap::KFUpdate(const Eigen::VectorXd& z, const double& var,
                       const Eigen::SparseMatrix<double>& H,
                       const std::string cmaes_objective) {
  // Compute the innovation. NB: Column-major ordering.
  Eigen::Map<const Eigen::MatrixXd> x(data_.data(), data_.rows() * data_.cols(),
                                      1);
  Eigen::MatrixXd v = z - H * x;
  Eigen::MatrixXd x_new;
  // Kalman filter update.
  Eigen::MatrixXd PHt = covariance_ * H.transpose();
  Eigen::MatrixXd S =
      Eigen::MatrixXd(H * PHt) +
      Eigen::MatrixXd((var * Eigen::MatrixXd::Ones(H.rows(), 1)).asDiagonal());
  const Eigen::LLT<Eigen::MatrixXd> llt_of_S(S);
  // Check if S is positive definite.
  if (llt_of_S.info() == Eigen::NumericalIssue) {
    x_new = x + PHt * S.inverse() * v;
    covariance_ = covariance_ - PHt * S.inverse() * H * covariance_;
  } else {
    Eigen::MatrixXd Wc = llt_of_S.matrixU().solve<Eigen::OnTheRight>(PHt);
    Eigen::MatrixXd W =
        llt_of_S.matrixU().transpose().solve<Eigen::OnTheRight>(Wc);
    // logger_.writeKFUpdateData(z, H, var, covariance_, data_);
    x_new = x + W * v;
    covariance_ = covariance_ - Wc * Wc.transpose();
  }
  x_new.resize(data_.rows(), data_.cols());
  data_ = x_new;

  if(cmaes_objective == "acquisition_EI")
    expectedImprovement();
  else if(cmaes_objective == "acquisition_PI")
    probabilityOfImprovement();
  else if(cmaes_objective == "acquisition_UCB")
    upperConfidenceBound();
}

void GridMap::KFUpdate(const double& var,
                       const Eigen::SparseMatrix<double>& H,
                       const std::string cmaes_objective) {
  Eigen::LLT<Eigen::MatrixXd> llt_of_S(
      Eigen::MatrixXd(H * covariance_ * H.transpose()) +
      Eigen::MatrixXd((var * Eigen::MatrixXd::Ones(H.rows(), 1)).asDiagonal()));
  // Check if S is positive definite.
  if (llt_of_S.info() == Eigen::NumericalIssue) {
    covariance_ =
        covariance_ -
        covariance_ * H.transpose() *
            (Eigen::MatrixXd(H * covariance_ * H.transpose()) +
             Eigen::MatrixXd(
                 (var * Eigen::MatrixXd::Ones(H.rows(), 1)).asDiagonal()))
                .inverse() *
            H * covariance_;
  } else {
    Eigen::MatrixXd Wc = llt_of_S.matrixU().solve<Eigen::OnTheRight>(
        covariance_ * H.transpose());
    covariance_ = covariance_ - Wc * Wc.transpose();
  }

  if(cmaes_objective == "acquisition_EI")
    expectedImprovement();
  else if(cmaes_objective == "acquisition_PI")
    probabilityOfImprovement();
  else if(cmaes_objective == "acquisition_UCB")
    upperConfidenceBound();
  //LOG(INFO)<<"Covariance size: "<<covariance_.rows()<<","<<covariance_.cols();
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

Eigen::SparseMatrix<double> GridMap::constructMeasurementModel(
    const Eigen::VectorXd& measurement_indices) const 
{
  Eigen::SparseMatrix<double> H(measurement_indices.rows(),
                                data_.rows() * data_.cols());
  // Identify and index the states corresponding to a measurement.
  // TODO: - Multiresolution measurements?
  for (size_t i = 0; i < measurement_indices.rows(); ++i) {
    H.insert(i, measurement_indices(i)) = 1.0;
  }
  return H;
}

// void GridMap::expectedImprovement(const std::shared_ptr<voxblox::EsdfMap>& esdf_map_)
// {
//   // EI for minimising the data
//   LOG(INFO)<<"Enter EI";
//   Eigen::MatrixXd sigma = computeSTD();
//   Eigen::MatrixXd Z;
//   Z.resize(length_(0), length_(1));
//   double f_x_plus = data_.maxCoeff();
//   // double f_x_plus = minDataWithoutObstacles(esdf_map_);
//   // LOG(INFO)<<"f_x_plus: "<<f_x_plus;

//   double Xi = 0.2; // 0.01 is exploitation  0.3 is exploration

//   boost::accumulators::accumulator_set<double, \
//         boost::accumulators::stats<boost::accumulators::tag::variance> > acc;
//   for (size_t j = 0; j < data_.cols(); ++j) 
//   {
//     for (size_t i = 0; i < data_.rows(); ++i) 
//     {
//       if(sigma(i,j)<sqrt(0.06))
//         Z(i,j) = 0;
//       else
//         Z(i,j) = (-data_(i,j)+f_x_plus-Xi)/sigma(i,j);

//       acc(Z(i,j));
//     }
//   }
//   double Z_mean = boost::accumulators::mean(acc);
//   double Z_std  = sqrt(boost::accumulators::variance(acc));

//   for (size_t j = 0; j < data_.cols(); ++j) 
//   {
//     for (size_t i = 0; i < data_.rows(); ++i) 
//     {
//       if(sigma(i,j)<sqrt(0.06))
//         acquisition(i,j) = 0;
//       else
//       {
//         // pnorm = normal CDF, dnorm = normal PDF
//         acquisition(i,j) = (-data_(i,j)+f_x_plus-Xi)*stats::pnorm(Z(i,j),Z_mean,Z_std)+\
//                         sigma(i,j)*stats::dnorm(Z(i,j),Z_mean,Z_std);
//       }
//     }
//   }
// LOG(INFO)<<"End of EI";
// }

void GridMap::probabilityOfImprovement()
{
  // PI for maximising the data
  // LOG(INFO)<<"Updating PI";
  Eigen::MatrixXd sigma = computeSTD();
  Eigen::MatrixXd Z;
  Z.resize(length_(0), length_(1));
  double f_x_plus = data_.maxCoeff();

  double Xi = -0.1; // positive values give negative acquisition
                    // -0.3 exploitation -0.1 more exploration

  boost::accumulators::accumulator_set<double, \
        boost::accumulators::stats<boost::accumulators::tag::variance> > acc;
  for (size_t j = 0; j < data_.cols(); ++j) 
  {
    for (size_t i = 0; i < data_.rows(); ++i) 
    {
      if(sigma(i,j)<sqrt(0.02))
        Z(i,j) = 0;
      else
        Z(i,j) = (data_(i,j)-f_x_plus-Xi)/sigma(i,j);

      acc(Z(i,j));
    }
  }
  double Z_mean = boost::accumulators::mean(acc);
  double Z_std  = sqrt(boost::accumulators::variance(acc));

  for (size_t j = 0; j < data_.cols(); ++j) 
  {
    for (size_t i = 0; i < data_.rows(); ++i) 
    {
      if(sigma(i,j)<sqrt(0.02))
        acquisition(i,j) = 0;
      else
      {
        // pnorm = normal CDF, dnorm = normal PDF
        acquisition(i,j) = stats::pnorm(Z(i,j),Z_mean,Z_std);
        // if(acquisition(i,j)<0)
        //   acquisition(i,j) = 0;   // bring negative acquisition values to 0
      }
    }
  }


}

void GridMap::expectedImprovement()
{
  // EI for maximising the data
  // LOG(INFO)<<"Updating EI";
  Eigen::MatrixXd sigma = computeSTD();
  Eigen::MatrixXd Z;
  Z.resize(length_(0), length_(1));
  double f_x_plus = data_.minCoeff();
  // double f_x_plus = minDataWithoutObstacles(esdf_map_);
  // LOG(INFO)<<"f_x_plus: "<<f_x_plus;

  double Xi = 0; // positive values give negative acquisition
                    // -0.3 exploitation -0.1 more exploration

  boost::accumulators::accumulator_set<double, \
        boost::accumulators::stats<boost::accumulators::tag::variance> > acc;
  for (size_t j = 0; j < data_.cols(); ++j) 
  {
    for (size_t i = 0; i < data_.rows(); ++i) 
    {
      if(sigma(i,j)<sqrt(0.02))
        Z(i,j) = 0;
      else
        Z(i,j) = (data_(i,j)-f_x_plus-Xi)/sigma(i,j);

      acc(Z(i,j));
    }
  }
  double Z_mean = boost::accumulators::mean(acc);
  double Z_std  = sqrt(boost::accumulators::variance(acc));

  for (size_t j = 0; j < data_.cols(); ++j) 
  {
    for (size_t i = 0; i < data_.rows(); ++i) 
    {
      if(sigma(i,j)<sqrt(0.02))
        acquisition(i,j) = 0;
      else
      {
        // pnorm = normal CDF, dnorm = normal PDF
        acquisition(i,j) = (data_(i,j)-f_x_plus-Xi)*stats::pnorm(Z(i,j),Z_mean,Z_std)+\
                        sigma(i,j)*stats::dnorm(Z(i,j),Z_mean,Z_std);
        // if(acquisition(i,j)<0)
        //   acquisition(i,j) = 0;   // bring negative acquisition values to 0
      }
    }
  }

  // Normalize the EI (do check the consequences of doing this)
  // acquisition = 0.5*(acquisition-Eigen::MatrixXd::Ones(acquisition.rows(),acquisition.cols())*\
  //                 acquisition.minCoeff())/(acquisition.maxCoeff()-acquisition.minCoeff());

// LOG(INFO)<<"End of EI";
}

void GridMap::upperConfidenceBound()
{
  Eigen::MatrixXd sigma = computeSTD();
  double Xi = 1;


  for (size_t j = 0; j < data_.cols(); ++j) 
  {
    for (size_t i = 0; i < data_.rows(); ++i) 
    {
      if(sigma(i,j)<sqrt(0.02))
        acquisition(i,j) = 0;
      else
      {
        acquisition(i,j) = data_(i,j)+Xi*sigma(i,j);
      }
    }
  } 
}

double GridMap::minDataWithoutObstacles(const std::shared_ptr<voxblox::EsdfMap>& esdf_map_)
{
  double result = std::numeric_limits<double>::infinity();
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
            if(dist<=-1) // Change this to variables
              continue;
            else
            {
              if(data_(i,j)<result)
                result = data_(i,j);
            }

          }
        else
          LOG(ERROR)<<"Point not found in the map! Inside minDataWithoutObstacles() ";
      }
    }
  return result;
}

Eigen::MatrixXd GridMap::computeSTD()
{
  Eigen::MatrixXd Std;
  Std.resize(length_(0), length_(1));
  Eigen::MatrixXd P_diag = covariance_.diagonal();
  Eigen::Map<const Eigen::MatrixXd> P(P_diag.data(), data_.rows(),
                                      data_.cols());
  for (size_t j = 0; j < data_.cols(); ++j) {
    for (size_t i = 0; i < data_.rows(); ++i) {
      Std(i,j) = sqrt(P(i,j));
    }
  }
  // LOG(INFO)<<"End of computeSTD";
  return Std;
}

double GridMap::computeSensorNoiseVariance(
    const double& camera_height, const double& sensor_coefficient_A,
    const double& sensor_coefficient_B) const {
  return sensor_coefficient_A *
         (1 - exp(-sensor_coefficient_B * camera_height));
  // return 0.05*(exp(0.15*camera_height));         
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

double GridMap::getDistanceBetweenPoints(const geometry_msgs::Point& p1,
                                             const geometry_msgs::Point& p2) {
  return sqrt(pow(p1.x - p2.x, 2.0) + pow(p1.y - p2.y, 2.0) +
              pow(p1.z - p2.z, 2.0));
}