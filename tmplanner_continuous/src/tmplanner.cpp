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

#include "tmplanner_continuous/tmplanner.h"

using namespace tmplanner;

tmPlanner::tmPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): \
  voxblox::EsdfServer(nh,nh_private), grid_map_(nh,nh_private)
  {
}


void tmPlanner::initializeObstacleMap()
{
  /////////// Initialisation for voxblox///////////////
   esdf_gt_mesh_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "esdf_gt_mesh", 1, true);

  srand (static_cast <unsigned> (time(0)));

  // Create the Esdf map
  config_.esdf_voxel_size = 0.1;
  config_.esdf_voxels_per_side = 16u;
  esdf_map_.reset(new voxblox::EsdfMap(config_));
  // esdf_gt_.reset(new voxblox::Layer<voxblox::EsdfVoxel>(config_.esdf_voxel_size, config_.esdf_voxels_per_side));

  esdf_gt_ = std::unique_ptr<voxblox::Layer<voxblox::EsdfVoxel> >(new voxblox::Layer<voxblox::EsdfVoxel>(config_.esdf_voxel_size, config_.esdf_voxels_per_side));

  // Set the environment boundary
  voxblox::Point min_bound_((-map_parameters_.width/2)-2.4,(-map_parameters_.height/2)-2.4,-.0001);
  voxblox::Point max_bound_((map_parameters_.width/2)+2.4,(map_parameters_.height/2)+2.4,planning_parameters_.maximum_height+2);  //15.4 and -15.4
  simulation_world_.setBounds(min_bound_,max_bound_);

  LOG(INFO)<<"Bounds are "<<simulation_world_.getMinBound();
  LOG(INFO)<<"Bounds are "<<simulation_world_.getMaxBound();
  /////////// Initialisation for voxblox///////////////

  buildWorld(0);  
}

void tmPlanner::buildWorld(voxblox::FloatingPoint plane_height)
{ 
    // Add ground
  LOG(INFO)<<"Building ground plane ";
  simulation_world_.addGroundLevel(plane_height);

  // Add a cylinder to the environment
  // voxblox::Point centre(-7,-7,12.5);
  // voxblox::FloatingPoint radius = 3, height = 25;
  // std::unique_ptr<voxblox::Object> object_ptr = std::unique_ptr<voxblox::Object>(new voxblox::Cylinder(centre,radius,height));
  // LOG(INFO)<<"Building cylinder 1";
  // simulation_world_.addObject(std::move(object_ptr));

  // Add a sphere to the environment
  // voxblox::Point centre2(0,-10,3);
  // voxblox::FloatingPoint radius2 = 3;
  // std::unique_ptr<voxblox::Object> object_ptr2 = std::unique_ptr<voxblox::Object>(new voxblox::Sphere(centre2,radius2));
  // LOG(INFO)<<"Building Sphere 1";
  // simulation_world_.addObject(std::move(object_ptr2));

  // voxblox::Point centre3(0,10,3);
  // voxblox::FloatingPoint radius3 = 3;
  // std::unique_ptr<voxblox::Object> object_ptr3 = std::unique_ptr<voxblox::Object>(new voxblox::Sphere(centre3,radius3));
  // LOG(INFO)<<"Building Sphere 2";
  // simulation_world_.addObject(std::move(object_ptr3));

  // Add a cube to the environment
  // voxblox::Point centre1(0,0,13); 
  // voxblox::Point size1(10,4,26); 
  // std::unique_ptr<voxblox::Object> object_ptr1 = std::unique_ptr<voxblox::Object>(new voxblox::Cube(centre1,size1));
  // LOG(INFO)<<"Building cube 1";
  // simulation_world_.addObject(std::move(object_ptr1));

  // voxblox::Point centre2(0,4,13); 
  // voxblox::Point size2(10,4,26); 
  // std::unique_ptr<voxblox::Object> object_ptr2 = std::unique_ptr<voxblox::Object>(new voxblox::Cube(centre2,size2));
  // LOG(INFO)<<"Building cube 2";
  // simulation_world_.addObject(std::move(object_ptr2));

  // voxblox::Point centre3(0,-4,13); 
  // voxblox::Point size3(10,4,26); 
  // std::unique_ptr<voxblox::Object> object_ptr3 = std::unique_ptr<voxblox::Object>(new voxblox::Cube(centre3,size3));
  // LOG(INFO)<<"Building cube 3";
  // simulation_world_.addObject(std::move(object_ptr3));

  // int no_of_rand_box = 5;
  // for(int i=0;i<no_of_rand_box;i++)
  //   addRandomBoxToSimulation();

  ///////////////////////// REAL WORLD MODEL ///////////////////

  // voxblox::Point centre2(-10,0,1.97/2), size2(5.84,2.35,1.97); 
  // std::unique_ptr<voxblox::Object> object_ptr2 = std::unique_ptr<voxblox::Object>(new voxblox::Cube(centre2,size2));
  // LOG(INFO)<<"Building Car";
  // simulation_world_.addObject(std::move(object_ptr2));

  // voxblox::Point centre3(-1,10,4.5);
  // voxblox::FloatingPoint radius3 = 4.5;
  // std::unique_ptr<voxblox::Object> object_ptr3 = std::unique_ptr<voxblox::Object>(new voxblox::Sphere(centre3,radius3));
  // LOG(INFO)<<"Building Tree 1";
  // simulation_world_.addObject(std::move(object_ptr3));

  // voxblox::Point centre4(-9,-9,4.5);
  // voxblox::FloatingPoint radius4 = 4.5;
  // std::unique_ptr<voxblox::Object> object_ptr4 = std::unique_ptr<voxblox::Object>(new voxblox::Sphere(centre4,radius4));
  // LOG(INFO)<<"Building Tree 2";
  // simulation_world_.addObject(std::move(object_ptr4));

  // voxblox::Point centre5(0,0,11.05/2), size5(10.1,9.01,11.05); 
  // std::unique_ptr<voxblox::Object> object_ptr5 = std::unique_ptr<voxblox::Object>(new voxblox::Cube(centre5,size5));
  // LOG(INFO)<<"Building House 1";
  // simulation_world_.addObject(std::move(object_ptr5));

  // voxblox::Point centre6(10,-8,3.5), size6(8.75,3.03,7); 
  // std::unique_ptr<voxblox::Object> object_ptr6 = std::unique_ptr<voxblox::Object>(new voxblox::Cube(centre6,size6));
  // LOG(INFO)<<"Building House 2";
  // simulation_world_.addObject(std::move(object_ptr6));

  // voxblox::Point centre7(10,8,3.5), size7(8.75,3.03,7); 
  // std::unique_ptr<voxblox::Object> object_ptr7 = std::unique_ptr<voxblox::Object>(new voxblox::Cube(centre7,size7));
  // LOG(INFO)<<"Building House 3";
  // simulation_world_.addObject(std::move(object_ptr7));

  // voxblox::Point centre8(-10,10,13.27/2);
  // voxblox::FloatingPoint radius8 = 2, height8 = 13.27;
  // std::unique_ptr<voxblox::Object> object_ptr8 = std::unique_ptr<voxblox::Object>(new voxblox::Cylinder(centre8,radius8,height8));
  // LOG(INFO)<<"Building reactor";
  // simulation_world_.addObject(std::move(object_ptr8));

  ///////////////////////// REAL WORLD MODEL ///////////////////

  ///////////////////////// ENVT FOR PAPER ///////////////////

  voxblox::Point centre2(-10,0,1.97/2), size2(5.84,2.35,1.97); 
  std::unique_ptr<voxblox::Object> object_ptr2 = std::unique_ptr<voxblox::Object>(new voxblox::Cube(centre2,size2));
  LOG(INFO)<<"Building Car";
  simulation_world_.addObject(std::move(object_ptr2));

  voxblox::Point centre5(0,0,11.05/2), size5(10.1,9.01,11.05); 
  std::unique_ptr<voxblox::Object> object_ptr5 = std::unique_ptr<voxblox::Object>(new voxblox::Cube(centre5,size5));
  LOG(INFO)<<"Building House 1";
  simulation_world_.addObject(std::move(object_ptr5));

  voxblox::Point centre6(10,-8,3.5), size6(8.75,3.03,7); 
  std::unique_ptr<voxblox::Object> object_ptr6 = std::unique_ptr<voxblox::Object>(new voxblox::Cube(centre6,size6));
  LOG(INFO)<<"Building House 2";
  simulation_world_.addObject(std::move(object_ptr6));

  voxblox::Point centre7(10,8,3.5), size7(8.75,3.03,7); 
  std::unique_ptr<voxblox::Object> object_ptr7 = std::unique_ptr<voxblox::Object>(new voxblox::Cube(centre7,size7));
  LOG(INFO)<<"Building House 3";
  simulation_world_.addObject(std::move(object_ptr7));

  ///////////////////////// ENVT FOR PAPER ///////////////////


  // // Add a cube to the environment
  // voxblox::Point centre4(7,7,10);
  // voxblox::Point size4(1.5,1.5,20);
  // std::unique_ptr<voxblox::Object> object_ptr4 = std::unique_ptr<voxblox::Object>(new voxblox::Cube(centre4,size4));
  // LOG(INFO)<<"Building cube 2";
  // simulation_world_.addObject(std::move(object_ptr4));

  // // Add a cube to the environment
  // voxblox::Point centre10(-7,7,10);
  // voxblox::Point size10(1.5,1.5,20);
  // std::unique_ptr<voxblox::Object> object_ptr10 = std::unique_ptr<voxblox::Object>(new voxblox::Cube(centre10,size10));
  // LOG(INFO)<<"Building cube 3";
  // simulation_world_.addObject(std::move(object_ptr10));

  // // Generate sdf
  LOG(INFO)<<"Generating Sdf ";
  voxblox::FloatingPoint max_dist = map_parameters_.width;
  simulation_world_.generateSdfFromWorld(max_dist,esdf_map_->getEsdfLayerPtr());
  esdf_gt_ = std::unique_ptr<voxblox::Layer<voxblox::EsdfVoxel> >(esdf_map_->getEsdfLayerPtr());
  // sliced_esdf_map(13);
  updateMapMesh();

  // Add boundary planes
  LOG(INFO)<<"Building boundary planes ";
  voxblox::Point centre9(-map_parameters_.width/2,0,planning_parameters_.maximum_height/2), normal9(1,0,0);
  std::unique_ptr<voxblox::Object> object_ptr9 = std::unique_ptr<voxblox::Object>(new voxblox::PlaneObject(centre9,normal9));
  simulation_world_.addObject(std::move(object_ptr9));

  voxblox::Point centre10(map_parameters_.width/2,0,planning_parameters_.maximum_height/2), normal10(-1,0,0);
  std::unique_ptr<voxblox::Object> object_ptr10 = std::unique_ptr<voxblox::Object>(new voxblox::PlaneObject(centre10,normal10));
  simulation_world_.addObject(std::move(object_ptr10));

  voxblox::Point centre11(0,-map_parameters_.height/2,planning_parameters_.maximum_height/2), normal11(0,1,0);
  std::unique_ptr<voxblox::Object> object_ptr11 = std::unique_ptr<voxblox::Object>(new voxblox::PlaneObject(centre11,normal11));
  simulation_world_.addObject(std::move(object_ptr11));

  voxblox::Point centre12(0,map_parameters_.height/2,planning_parameters_.maximum_height/2), normal12(0,-1,0);
  std::unique_ptr<voxblox::Object> object_ptr12 = std::unique_ptr<voxblox::Object>(new voxblox::PlaneObject(centre12,normal12));
  simulation_world_.addObject(std::move(object_ptr12));  

  voxblox::Point centre13(0,0,planning_parameters_.maximum_height), normal13(0,0,-1);
  std::unique_ptr<voxblox::Object> object_ptr13 = std::unique_ptr<voxblox::Object>(new voxblox::PlaneObject(centre13,normal13));
  simulation_world_.addObject(std::move(object_ptr13)); 

  // // Done so that rviz does not have the boundary planes covering the environment anymore
  esdf_map_.reset(new voxblox::EsdfMap(config_));
  simulation_world_.generateSdfFromWorld(max_dist,esdf_map_->getEsdfLayerPtr());
  esdf_gt_.reset(nullptr);
  // esdf_gt_.reset(new voxblox::Layer<voxblox::EsdfVoxel>(config_.esdf_voxel_size, config_.esdf_voxels_per_side));
  esdf_gt_ = std::unique_ptr<voxblox::Layer<voxblox::EsdfVoxel> >(esdf_map_->getEsdfLayerPtr()); 

  // Generate sdf
  // LOG(INFO)<<"Generating Sdf ";
  // voxblox::FloatingPoint max_dist = map_parameters_.width;
  // simulation_world_.generateSdfFromWorld(max_dist,esdf_map_->getEsdfLayerPtr());
  // esdf_gt_ = std::unique_ptr<voxblox::Layer<voxblox::EsdfVoxel> >(esdf_map_->getEsdfLayerPtr());
  // // sliced_esdf_map(10);
  // updateMapMesh();

  //testFunctions();
}

void tmPlanner::addRandomBoxToSimulation()
{
  voxblox::Point centre1; 
  voxblox::Point size1(4,4,26); 
  float x, y;
  do
  {
    voxblox::Point min_bound_(-(map_parameters_.width/2)+(size1[0]/2),-(map_parameters_.height/2)+(size1[1]/2),0);
    voxblox::Point max_bound_((map_parameters_.width/2)-(size1[0]/2),(map_parameters_.height/2)-(size1[1]/2),0);

    x = min_bound_[0] + static_cast <float> (rand()) / \
                    ( static_cast <float> (RAND_MAX/(max_bound_[0]-min_bound_[0])));
    y = min_bound_[1] + static_cast <float> (rand()) / \
                    ( static_cast <float> (RAND_MAX/(max_bound_[1]-min_bound_[1])));
  }while(sqrt(pow(planning_parameters_.initial_position.x - x,2)+pow(planning_parameters_.initial_position.y - y,2)) < (sqrt(pow(size1[0],2)+pow(size1[1],2))/2)+0.5);

  centre1 << x,y,size1[2]/2;
  std::unique_ptr<voxblox::Object> object_ptr1 = std::unique_ptr<voxblox::Object>(new voxblox::Cube(centre1,size1));
  LOG(INFO)<<"Building random cube";
  simulation_world_.addObject(std::move(object_ptr1));
}

void tmPlanner::testFunctions()
{
  LOG(INFO)<<"Testing obstacle cost: "<<obstacleCost(0,"logistic");
  geometry_msgs::Point p1, p2;
  p1.x = -12.0288; p1.y = -4.4671, p1.z = 23.6581;
  p2.x = 6.40098; p2.y = 1.44886, p2.z = 23.0064;
  LOG(INFO)<<"Testing isObstacleFreePath(): "<<isObstacleFreePath(p1,p2);

  // Read out the esdf value at a point
  Eigen::Vector3d pos(-5.31894,0.723279,24.5389);
  double dist;
  if(esdf_map_->getDistanceAtPosition(pos,&dist))
  {
    LOG(INFO)<<"Distance at position "<<pos<<" is "<<dist;
  }

  control_poses_.clear();
  trajectory_.clear();
  geometry_msgs::Pose test_pose_;
  test_pose_.position.x = 12.2008;
  test_pose_.position.y = 6.65963;
  test_pose_.position.z = 20.0152;

  // test_pose_.position.x = 0;
  // test_pose_.position.y = -10;
  // test_pose_.position.z = 13;

  test_pose_.orientation.x = -0.0423476;
  test_pose_.orientation.y = -0.0205449;
  test_pose_.orientation.z = 0.831907;
  control_poses_.push_back(test_pose_);

  test_pose_.position.x = -0.488325;
  test_pose_.position.y = -12.4194;
  test_pose_.position.z = 23.4608;

  // test_pose_.position.x = 0;
  // test_pose_.position.y = 10;
  // test_pose_.position.z = 13;

  test_pose_.orientation.x = 0;
  test_pose_.orientation.y = 0;
  test_pose_.orientation.z = -0.806606;
  control_poses_.push_back(test_pose_);

  test_pose_.position.x = -5.31894;
  test_pose_.position.y = 0.723279;
  test_pose_.position.z = 24.5389;

  // test_pose_.position.x = 4;
  // test_pose_.position.y = 10;
  // test_pose_.position.z = 13;
  
  test_pose_.orientation.x = 0;
  test_pose_.orientation.y = 0;
  test_pose_.orientation.z = -0.806606;
  control_poses_.push_back(test_pose_);
  createPolynomialTrajectory();

  LOG(INFO)<<"Is collision free trajectory? "<<isCollisionFreeTrajectory();
}

void tmPlanner::sliced_esdf_map(voxblox::FloatingPoint plane_height)
{
  float map_incr = .2;
  voxblox::Point min_bound_ = simulation_world_.getMinBound();
  voxblox::Point max_bound_ = simulation_world_.getMaxBound();

  int x = std::floor((max_bound_[0]-min_bound_[0]-map_incr)/map_incr);
  int y = std::floor((max_bound_[1]-min_bound_[1]-map_incr)/map_incr);
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> map;
  map.resize(x-4,y-4);
  ROS_INFO_STREAM("x = "<<x<<" y = "<<y);

  // Evaluate esdf at each point on the plane (discretized)
  double dist;
  for(int i = 1; i <= x-4; i++)
  {
    for(int j = 1; j <= y-4; j++)
    {
      Eigen::Vector3d pos(min_bound_[0]+i*map_incr,min_bound_[1]+j*map_incr,plane_height);
      if(esdf_map_->getDistanceAtPosition(pos,&dist))
        map(i-1,j-1) = dist;
    }
  }


  saveMatrixToCSV(map);

  // Eigen::Vector3d pos(9,9,1);
  // double dist;
  // if(esdf_map_->getDistanceAtPosition(pos,&dist))
  // {
  //  ROS_INFO_STREAM("Distance at position is "<<dist);
  // }

}

void tmPlanner::saveMatrixToCSV(Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> map)
{
  std::ofstream f;
  f.open(ros::package::getPath("tmplanner_continuous") + "/debug/matrices/slicemap" + ".csv");
  LOG(INFO)<<"Saving the map ";
  f << map.format(csv_format) << '\n';
  f.close();
}

void tmPlanner::updateMapMesh()
{
    
    voxblox::MeshIntegratorConfig mesh_config;
    voxblox::MeshLayer::Ptr mesh(new voxblox::MeshLayer(esdf_gt_->block_size()));
    voxblox::MeshIntegrator<voxblox::EsdfVoxel> mesh_integrator(mesh_config, esdf_gt_.get(),
                                              mesh.get());

    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator.generateMesh(only_mesh_updated_blocks, clear_updated_flag);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(1);
    voxblox::ColorMode color_mode = voxblox::ColorMode::kNormals;
    voxblox::fillMarkerWithMesh(mesh, color_mode, &marker_array.markers[0]);
    marker_array.markers[0].header.frame_id = world_frame_;

    LOG(INFO)<<"Publishing esdf mesh ";
    esdf_gt_mesh_pub_.publish(marker_array);

}


void tmPlanner::setupPlanner() {
  elapsed_time_ = 0.0;
  control_poses_.clear();
  trajectory_.clear();

  // Build the world with obstacles
  initializeObstacleMap();

  // Set up the grid map.
  grid_map_.setMapGeometry(
      map_parameters_.width, map_parameters_.height,
      map_parameters_.resolution_x, map_parameters_.resolution_y,
      -map_parameters_.width / 2.0, -map_parameters_.height / 2.0,
      map_parameters_.frame_id,sensor_parameters_.sensor_type);
  grid_map_.fillUnknown();
  grid_map_.computeCovariance(
      map_parameters_.cov_fun, map_parameters_.log_hyperparams,
      map_parameters_.resolution_x, map_parameters_.resolution_y, esdf_map_,
      optimization_parameters_.cmaes_objective);

  // Set up the lattice for 3-D grid search.
  lattice_.createLattice(
      planning_parameters_.maximum_height, planning_parameters_.minimum_height,
      sensor_parameters_.fov_angle_x, sensor_parameters_.fov_angle_y,
      planning_parameters_.lattice_min_height_points,
      planning_parameters_.lattice_height_increment, grid_map_);
  removeLatticePoints();
  
  if (optimization_parameters_.optimization_method == "coverage_planning")
  {
    ros::ServiceClient client = nh_private_.serviceClient<planning_msgs::PlannerService>("/mav_coverage_planning/plan_path");
    coverage_srv.request.start_pose.pose.position = planning_parameters_.initial_position;
    coverage_srv.request.goal_pose.pose.position = planning_parameters_.initial_position;

    // set orientation to 0.0.
    coverage_srv.request.start_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    coverage_srv.request.goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    if (client.call(coverage_srv))
    {
      LOG(INFO)<<"Coverage planner path ready! "<<coverage_srv.response.sampled_plan.points[0].transforms[0].translation;
      coverage_point = 1;
    }
    else
      ROS_ERROR("Failed to call service plan_path");

  }
  else if(optimization_parameters_.optimization_method == "testing")
  {
    testing_path.clear();
    geometry_msgs::Pose next_point;
    next_point.position.x = planning_parameters_.initial_position.x;
    next_point.position.y = planning_parameters_.initial_position.y;
    next_point.position.z = planning_parameters_.initial_position.z;//planning_parameters_.minimum_height;
    next_point.orientation = tf::createQuaternionMsgFromYaw(0.0);
    testing_path.push_back(next_point);
    int i = 1;
    while(next_point.position.z < planning_parameters_.maximum_height)
    {
      next_point.position.z = next_point.position.z + 0.1;
      testing_path.push_back(next_point);
      i++;
    }
    LOG(INFO)<<"Testing planner path ready! ";
    coverage_point = 1;

  }

  if(sensor_parameters_.sensor_type == "human")
  {
    const human_detection::DetectorMethod detector_method = human_detection::DetectorMethod::YOLO;
    if (human_detection::HumanDetectorFactory::createDetector(detector_method, nh_, nh_private_,
                                             human_detector_ptr)) {
    } else {
      LOG(INFO) << "Could not create a human detector.";
      ros::requestShutdown();
    }
    LOG(INFO) << "Human detector loaded";

    // Mark the beginning of flight for classifier metrics
    std::ofstream f;
    f.open(ros::package::getPath("tmplanner_continuous") +
             "/debug/classifier_metrics.dat",std::ofstream::app);
    CHECK(f.is_open()) << "Error opening file!";
    f<<"height,recall,precision,F1_score,RMSE;\n";
    f.close();
  }

  if(optimization_parameters_.cmaes_objective == "covariance")
  {
    logger_.writeStartToFile("cov_traces");
    logger_.writeStartToFile("time_trace_traj");
  }
  // else if(optimization_parameters_.cmaes_objective=="acquisition_EI" || optimization_parameters_.cmaes_objective=="acquisition_PI")

  logger_.writeStartToFile("metric");
  logger_.writeStartToFile("metric_traj");
}

void tmPlanner::removeLatticePoints()
{
  double dist;
  std::vector<geometry_msgs::Point> lattice_points = lattice_.getLatticePoints();
  LOG(INFO)<<"Lattice points before: "<<lattice_points.size();
  for (std::vector<geometry_msgs::Point>::iterator i=lattice_points.begin(); i!=lattice_points.end();) 
  {
    
    Eigen::Vector3d pos(i->x,i->y,i->z);
    if(esdf_map_->getDistanceAtPosition(pos,&dist))
    {
      if(dist<=0)
      {
        i = lattice_points.erase(i);
        LOG(INFO)<<"Erased Lattice point: ("<<pos[0]<<","<<pos[1]<<","<<pos[2]<<"), dist: "<<dist;
      }
      else
      {
        ++i;
        //LOG(INFO)<<"Not erasing, ("<<pos[0]<<","<<pos[1]<<","<<pos[2]<<"), dist: "<<dist;
      }
    }
    else
      LOG(ERROR)<<"The point ("<<pos[0]<<","<<pos[1]<<","<<pos[2] \
        <<") doesn't exist in the map or is not observed! in removeLatticePoints()";
    
  }
  lattice_.setLatticePoints(lattice_points);
  LOG(INFO)<<"Lattice points after: "<<lattice_.getLatticePoints().size();

}

bool tmPlanner::isBudgetSpent() {
  elapsed_time_ = ros::Time::now().toSec();
  //LOG(INFO)<<"Elapsed time: "<<elapsed_time_;
  if (elapsed_time_ > planning_parameters_.time_budget) {
    LOG(INFO) << "Budget spent. Exiting...";
    return true;
  } else {
    return false;
  }
}

bool tmPlanner::createInitialPlan() {
  testFunctions();

  control_poses_.clear();
  trajectory_.clear();
  initial_run_time = ros::Time::now().toSec();

  // Compute current world to vicon-sensor-body transform.
  kindr::minimal::QuatTransformation T_MAP_VSB;
  tf::poseMsgToKindr(odometry_pose_, &T_MAP_VSB);
  kindr::minimal::QuatTransformation T_W_VSB =
      map_parameters_.T_W_MAP * T_MAP_VSB;
  geometry_msgs::Pose odometry_pose_world;
  tf::poseKindrToMsg(T_W_VSB, &odometry_pose_world);
  control_poses_.push_back(odometry_pose_world);
  geometry_msgs::Pose target_pose;
  target_pose.position = planning_parameters_.initial_position;
  // For now, set orientation to 0.0.
  target_pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  control_poses_.push_back(target_pose);
  createPolynomialTrajectory();

  // Write and display initial metrics (variance or acquisition)
  if(optimization_parameters_.cmaes_objective=="covariance")
  {
    double init_trace = grid_map_.computeCovarianceTrace();
    LOG(INFO)<<"Initial trace: "<<init_trace;
    logger_.writeCovarianceTrace(ros::Time::now().toSec()-initial_run_time,init_trace);

    std::ofstream f;
    f.open(ros::package::getPath("tmplanner_continuous") +
             "/debug/time_trace_traj.dat",std::ofstream::app);
    CHECK(f.is_open()) << "Error opening file!";
    f << ros::Time::now().toSec()-initial_run_time << "," << init_trace << ";\n";
    f.close();
  }
  // else if(optimization_parameters_.cmaes_objective=="acquisition_EI" || optimization_parameters_.cmaes_objective=="acquisition_PI")
  double init_metric;
  if(optimization_parameters_.error_metric == "RMSE")
  {
    // LOG(INFO)<<"Initial acquisition sum: "<<grid_map_.getAcquisitionSum();
    init_metric = grid_map_.calculateRMSE();
    LOG(INFO)<<"Initial RMSE: "<<init_metric;
  }
  else if(optimization_parameters_.error_metric == "WRMSE")
  {
    init_metric = grid_map_.calculateWRMSE();
    LOG(INFO)<<"Initial WRMSE: "<<init_metric;
  }
  else
    ROS_ERROR("Error metric not initialized correctly");

  logger_.writeLineToFile("metric",ros::Time::now().toSec()-initial_run_time,init_metric);
  logger_.writeLineToFile("metric_traj",ros::Time::now().toSec()-initial_run_time,init_metric);

  return true;
}

bool tmPlanner::createNewPlan(const geometry_msgs::Pose& current_pose) 
{
  if (optimization_parameters_.optimization_method == "cmaes")
  {
    do
    {
      control_poses_.clear();
      // STEP 1. Grid search on the lattice.
      std::chrono::steady_clock::time_point begin =
          std::chrono::steady_clock::now();

      // Landing criteria - no next best point can be found 
      if (!searchGrid(current_pose))
            return false;    

      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      LOG(INFO) << "Search time = "
                << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                         begin)
                       .count();

      // additional code for checking final soultion                 
      LOG(INFO)<<"Control poses before optimization: ";
      for (auto pose : control_poses_) 
      {
        LOG(INFO)<<"        "<<pose.position.x<<","<<pose.position.y<<","\
                <<pose.position.z<<" ";
      }
      // end additional code for checking final soultion

      // STEP 2. Evolutionary optimization.
      optimizePathOnHorizon();
      createPolynomialTrajectory();
    }while(!isCollisionFreeTrajectory());

    LOG(INFO)<<"Control poses after optimization: ";
    for (auto pose : control_poses_) 
      LOG(INFO)<<"   ["<<pose.position.x<<","<<pose.position.y<<","\
              <<pose.position.z<<"]";
  }

  else if(optimization_parameters_.optimization_method == "bayesian_opt")
  {
    do
    {
      // control_poses_.clear();
      optimizePathOnHorizon();     
      createPolynomialTrajectory();

    }while(!isCollisionFreeTrajectory());

  }

  else if (optimization_parameters_.optimization_method == "random_sampling")
  {
    control_poses_.clear();
    control_poses_.push_back(current_pose);

    voxblox::Point min_bound_(-map_parameters_.width/2,-map_parameters_.height/2,planning_parameters_.minimum_height);
    voxblox::Point max_bound_(map_parameters_.width/2,map_parameters_.height/2,planning_parameters_.maximum_height);
    geometry_msgs::Pose next_pose;


    // Landing criteria - no next best point can be found on the lattice grid search
    // if (!landingCriteria(current_pose))
    //       return false;    
          
    do
    {
      next_pose.position.x = min_bound_[0] + static_cast <float> (rand()) / \
                ( static_cast <float> (RAND_MAX/(max_bound_[0]-min_bound_[0])));
      next_pose.position.y = min_bound_[1] + static_cast <float> (rand()) / \
                ( static_cast <float> (RAND_MAX/(max_bound_[1]-min_bound_[1])));
      // next_pose.position.z = 13;
      next_pose.position.z = min_bound_[2] + static_cast <float> (rand()) / \
                ( static_cast <float> (RAND_MAX/(max_bound_[2]-min_bound_[2])));
      LOG(INFO)<<"Sampled point: ("<<next_pose.position.x<<","<<\
            next_pose.position.y<<","<<next_pose.position.z<<")";
     }while(!isObstacleFreePath(current_pose.position,next_pose.position));

    next_pose.orientation = current_pose.orientation;
    control_poses_.push_back(next_pose);
    createPolynomialTrajectory();

    LOG(INFO)<<"Let's next visit : ("<<next_pose.position.x<<","<<\
            next_pose.position.y<<","<<next_pose.position.z<<")";
  }
  else if (optimization_parameters_.optimization_method == "coverage_planning")
  {
    control_poses_.clear();
    control_poses_.push_back(current_pose);

    // Landing criteria - no next best point can be found on the lattice grid search
    // if (!landingCriteria(current_pose))
    //       return false;    
          
    geometry_msgs::Pose next_pose;
    next_pose.position.x = coverage_srv.response.sampled_plan.points[coverage_point].transforms[0].translation.x;
    next_pose.position.y = coverage_srv.response.sampled_plan.points[coverage_point].transforms[0].translation.y;
    next_pose.position.z = coverage_srv.response.sampled_plan.points[coverage_point].transforms[0].translation.z;
    next_pose.orientation = current_pose.orientation;
    LOG(INFO)<<"Let's next visit : ("<<next_pose.position.x<<","<<\
            next_pose.position.y<<","<<next_pose.position.z<<")";

    control_poses_.push_back(next_pose);
    coverage_point++;
    createPolynomialTrajectory();
  }
  else if (optimization_parameters_.optimization_method == "testing")
  {
    control_poses_.clear();
    control_poses_.push_back(current_pose);

    geometry_msgs::Pose next_pose;
    next_pose = testing_path.at(coverage_point-1);

    LOG(INFO)<<"Let's next visit : "<<next_pose.position;
    control_poses_.push_back(next_pose);
    coverage_point++;
    createPolynomialTrajectory();

    // std::this_thread::sleep_for(std::chrono::seconds(7));  // delay for human detector to work at the point
  }
  else if(optimization_parameters_.optimization_method == "greedy")
  {
    // Initialize variables based on the current map state.
    // control_poses_.clear();
    // control_poses_.push_back(current_pose);
    // grid_map::GridMap simulated_grid_map = grid_map_;

    // LOG(INFO)<<"Evaluating the greedy solution";

    // // Find the next greedy point 
    // auto best_pose = findNextBestGridPointRandom(simulated_grid_map, current_pose);
    // // No next best point can be found (out-of-bounds result).
    // if (best_pose.position.x == 1000.0 && best_pose.position.y ==
    //         1000.0 && best_pose.position.z == -1.0) {
    //   return false;
    // }
    // control_poses_.push_back(best_pose);
    // LOG(INFO)<<"Let's next visit : "<<best_pose.position;
    // createPolynomialTrajectory();

    do
    {
      control_poses_.clear();
      control_poses_.push_back(current_pose);
      auto previous_pose = current_pose;
      grid_map::GridMap simulated_grid_map = grid_map_;

      // Perform the grid search.

      LOG(INFO)<<"Evaluating the greedy solution";

      while (control_poses_.size() < planning_parameters_.control_poses) 
      {
        // Find the next best point on the lattice.
        auto best_pose = findNextBestGridPointRandom(simulated_grid_map, previous_pose);
        // No next best point can be found (out-of-bounds result).
        if (best_pose.position.x == 1000.0 && best_pose.position.y ==
                1000.0 && best_pose.position.z == -1.0) {
          return false;
        }
        // Simulate a measurement at this point.
        bool if_kfupdate = true;
        simulated_grid_map.predictMapUpdate(
            best_pose, sensor_parameters_.T_IMU_CAM, sensor_parameters_.fov_angle_x,
            sensor_parameters_.fov_angle_y, sensor_parameters_.coefficient_A,
            sensor_parameters_.coefficient_B,esdf_map_,
            map_parameters_.width,map_parameters_.height,
            planning_parameters_.minimum_height,planning_parameters_.maximum_height,
            optimization_parameters_.cmaes_objective,if_kfupdate);
        control_poses_.push_back(best_pose);
        previous_pose = best_pose;
        if(optimization_parameters_.cmaes_objective=="covariance")
          LOG(INFO) << "Trace of P: " << simulated_grid_map.computeCovarianceTrace();
        else if(optimization_parameters_.cmaes_objective=="acquisition_EI" || optimization_parameters_.cmaes_objective=="acquisition_PI"
             || optimization_parameters_.cmaes_objective == "acquisition_UCB")
          LOG(INFO) << "Acquisition view : " << simulated_grid_map.getAcquisitionView();
      }
      createPolynomialTrajectory();

    }while(!isCollisionFreeTrajectory());

  }
  else
    LOG(INFO)<<"Planner name is not set correctly in the parameter file!";

  logger_.writeControlPoses(control_poses_);  
  // savePlannedTrajectory();
  return true;
}

void tmPlanner::savePlannedTrajectory()
{
  std::ofstream f;
  f.open(
      ros::package::getPath("tmplanner_continuous") + "/debug/planned_path.dat",
      std::ofstream::app);
  CHECK(f.is_open()) << "Error opening file!";

  mav_msgs::EigenTrajectoryPoint::Vector sampled_points;
  float sampling_frequency = 1;
  mav_trajectory_generation::sampleWholeTrajectory(
      trajectory_, 1.0 / sampling_frequency, &sampled_points);

  for (const auto& measurement_state : sampled_points) 
    f<< measurement_state.position_W[0] <<","<<measurement_state.position_W[1]<<","<<measurement_state.position_W[2]<< ";\n";

  f.close();   
}

bool tmPlanner::isCollisionFreeTrajectory()
{
  mav_msgs::EigenTrajectoryPoint::Vector obst_check_points;
  float obst_check_frequency = 4;
  mav_trajectory_generation::sampleWholeTrajectory(
      trajectory_, 1.0 / obst_check_frequency, &obst_check_points);

  double dist;
  bool flag_coll = false, flag_boundary = false;
  for (const auto& measurement_state : obst_check_points) 
  {
    
    if(esdf_map_->getDistanceAtPosition(measurement_state.position_W,&dist))
    {
       //LOG(INFO)<<"Cost = "<<obstacleCost(dist,"logistic");
      //LOG(INFO)<<"Dist: "<<dist;
      if(dist<0.5)
      {
        flag_coll = true;
        break;
      }
    }
    else
      LOG(ERROR)<<"The point doesn't exist in the map or is not observed! Inside isCollisionFreeTrajectory() ";
    
    geometry_msgs::Point measurement_state_copy;      // same data type bullshit again
    measurement_state_copy.x = measurement_state.position_W[0];
    measurement_state_copy.y = measurement_state.position_W[1];
    measurement_state_copy.z = measurement_state.position_W[2];
    if(!(grid_map_.insideBoundary(map_parameters_.width,map_parameters_.height,
      planning_parameters_.minimum_height,planning_parameters_.maximum_height,measurement_state_copy)))
      {
        flag_boundary = true;
        break;
      }
  } 
  LOG(INFO)<<"New path collides? = "<<flag_coll;
  LOG(INFO)<<"New path outside boundary? = "<<flag_boundary;
  return (!flag_coll && !flag_boundary);
}

void tmPlanner::createLandingPlan() {
  control_poses_.clear();
  control_poses_.push_back(odometry_pose_);
  geometry_msgs::Pose landing_pose;
  landing_pose.position.x = odometry_pose_.position.x;
  landing_pose.position.y = odometry_pose_.position.y;
  landing_pose.position.z = 0.05;
  landing_pose.orientation = odometry_pose_.orientation;
  control_poses_.push_back(landing_pose);
  createPolynomialTrajectory();
}

void tmPlanner::updateMap(const cv_bridge::CvImagePtr& cv_image_ptr,
                            const geometry_msgs::Pose& odometry_pose) {
  LOG(INFO) << "Updating map at x = " << odometry_pose.position.x
            << ", y = " << odometry_pose.position.y
            << ", z = " << odometry_pose.position.z;
  std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();
  grid_map_.updateMapFromImage(
      cv_image_ptr, odometry_pose, sensor_parameters_.T_IMU_CAM,
      sensor_parameters_.fov_angle_x, sensor_parameters_.fov_angle_y,
      sensor_parameters_.intrinsics_matrix, sensor_parameters_.coefficient_A,
      sensor_parameters_.coefficient_B,esdf_map_,map_parameters_.width,map_parameters_.height,
      planning_parameters_.minimum_height,planning_parameters_.maximum_height,
      optimization_parameters_.cmaes_objective,sensor_parameters_.sensor_type,
      human_detector_ptr, ros::Time::now().toSec()-initial_run_time);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  // LOG(INFO) << "Map update time = "
  //           << std::chrono::duration_cast<std::chrono::microseconds>(end -
  //                                                                    begin)
  //                  .count();

  logger_.writeMeasurementPose(odometry_pose);
  double running_time = ros::Time::now().toSec()-initial_run_time;
  if(optimization_parameters_.cmaes_objective == "covariance")
    logger_.writeCovarianceTrace(running_time,grid_map_.computeCovarianceTrace());
  // else if(optimization_parameters_.cmaes_objective == "acquisition_EI" || optimization_parameters_.cmaes_objective == "acquisition_PI")
  if(optimization_parameters_.error_metric == "RMSE")
  {
    double RMSE = grid_map_.calculateRMSE();
    LOG(INFO)<<"RMSE: "<<RMSE;
    logger_.writeLineToFile("metric",running_time,RMSE);
  }
  else if(optimization_parameters_.error_metric == "WRMSE")
  {
    double WRMSE = grid_map_.calculateWRMSE();
    LOG(INFO)<<"WRMSE: "<<WRMSE; 
    logger_.writeLineToFile("metric",running_time,WRMSE);
  }
  LOG(INFO)<<"Current run time: "<<running_time;
}

void tmPlanner::createPolynomialTrajectory() {
  CHECK_GT(control_poses_.size(), 0) << "No control poses!";

  trajectory_.clear();
  mav_trajectory_generation::Vertex::Vector path_vertices;
  mav_trajectory_generation::Vertex::Vector yaw_vertices;

  // Create the vertex lists for the planned path.
  for (std::deque<geometry_msgs::Pose>::size_type i = 0;
       i < control_poses_.size(); ++i) {
    mav_trajectory_generation::Vertex vertex(kDimensions);
    mav_trajectory_generation::Vertex yaw(1);

    if (i == 0 || i == control_poses_.size() - 1) {
      vertex.makeStartOrEnd(Eigen::Vector3d(control_poses_[i].position.x,
                                            control_poses_[i].position.y,
                                            control_poses_[i].position.z),
                            kDerivativetoOptimize);
    } else {
      vertex.addConstraint(
          mav_trajectory_generation::derivative_order::POSITION,
          Eigen::Vector3d(control_poses_[i].position.x,
                          control_poses_[i].position.y,
                          control_poses_[i].position.z));
    }
    path_vertices.push_back(vertex);

    yaw.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION,
                      tf::getYaw(control_poses_[i].orientation));
    yaw_vertices.push_back(yaw);
  }

  // Create a middle waypoint if there are only two.
  if (control_poses_.size() <= 2) {
    geometry_msgs::Pose wpa = control_poses_[0];
    geometry_msgs::Pose wpb = control_poses_[1];
    mav_trajectory_generation::Vertex midpoint_vertex(kDimensions);
    mav_trajectory_generation::Vertex midpoint_yaw(1);

    midpoint_vertex.addConstraint(
        mav_trajectory_generation::derivative_order::POSITION,
        Eigen::Vector3d(
            wpa.position.x + (wpb.position.x - wpa.position.x) / 2.0,
            wpa.position.y + (wpb.position.y - wpa.position.y) / 2.0,
            wpa.position.z + (wpb.position.z - wpa.position.z) / 2.0));
    midpoint_yaw.addConstraint(
        mav_trajectory_generation::derivative_order::ORIENTATION,
        tf::getYaw(wpa.orientation) +
            (tf::getYaw(wpb.orientation) - tf::getYaw(wpa.orientation)) / 2.0);
    path_vertices.insert(path_vertices.begin() + 1, midpoint_vertex);
    yaw_vertices.insert(yaw_vertices.begin() + 1, midpoint_yaw);
  }

  // Optimize polynomial trajectory.
  std::vector<double> segment_times;
  mav_trajectory_generation::Trajectory path_trajectory, yaw_trajectory;
  // Position.
  segment_times = estimateSegmentTimes(
      path_vertices, planning_parameters_.reference_speed,
      planning_parameters_.reference_acceleration, kFabianConstant);
  mav_trajectory_generation::PolynomialOptimization<kPolynomialCoefficients>
      opt(kDimensions);
  opt.setupFromVertices(path_vertices, segment_times, kDerivativetoOptimize);
  opt.solveLinear();
  opt.getTrajectory(&path_trajectory);
  // Yaw.
  mav_trajectory_generation::PolynomialOptimization<kPolynomialCoefficients>
      yaw_opt(1);
  yaw_opt.setupFromVertices(yaw_vertices, segment_times, kDerivativetoOptimize);
  yaw_opt.solveLinear();
  yaw_opt.getTrajectory(&yaw_trajectory);
  path_trajectory.getTrajectoryWithAppendedDimension(yaw_trajectory,
                                                     &trajectory_);
}

bool tmPlanner::landingCriteria(const geometry_msgs::Pose& initial_pose) 
{
  // Initialize variables based on the current map state.
  control_poses_.clear();
  control_poses_.push_back(initial_pose);
  auto previous_pose = initial_pose;
  grid_map::GridMap simulated_grid_map = grid_map_;

  // Perform the grid search.

  LOG(INFO)<<"Evaluating the best intial solution";

  while (control_poses_.size() < planning_parameters_.control_poses) {
    // Find the next best point on the lattice.
    auto best_pose = findNextBestGridPoint(simulated_grid_map, previous_pose);
    // No next best point can be found (out-of-bounds result).
    if (best_pose.position.x == 1000.0 && best_pose.position.y ==
            1000.0 && best_pose.position.z == -1.0) {
      return false;
    }
    // Simulate a measurement at this point.
    bool if_kfupdate = true;
    simulated_grid_map.predictMapUpdate(
        best_pose, sensor_parameters_.T_IMU_CAM, sensor_parameters_.fov_angle_x,
        sensor_parameters_.fov_angle_y, sensor_parameters_.coefficient_A,
        sensor_parameters_.coefficient_B,esdf_map_,
        map_parameters_.width,map_parameters_.height,
        planning_parameters_.minimum_height,planning_parameters_.maximum_height,
        optimization_parameters_.cmaes_objective,if_kfupdate);
    control_poses_.push_back(best_pose);
    previous_pose = best_pose;
    LOG(INFO) << "Trace of P: " << simulated_grid_map.computeCovarianceTrace();
  }
  return true;
}

bool tmPlanner::searchGrid(const geometry_msgs::Pose& initial_pose) 
{
  // Initialize variables based on the current map state.
  control_poses_.clear();
  control_poses_.push_back(initial_pose);
  auto previous_pose = initial_pose;
  grid_map::GridMap simulated_grid_map = grid_map_;

  // Perform the grid search.

  LOG(INFO)<<"Evaluating the best intial solution";

  while (control_poses_.size() < planning_parameters_.control_poses) {
    // Find the next best point on the lattice.
    auto best_pose = findNextBestGridPointRandom(simulated_grid_map, previous_pose);
    // No next best point can be found (out-of-bounds result).
    if (best_pose.position.x == 1000.0 && best_pose.position.y ==
            1000.0 && best_pose.position.z == -1.0) {
      return false;
    }
    // Simulate a measurement at this point.
    bool if_kfupdate = true;
    simulated_grid_map.predictMapUpdate(
        best_pose, sensor_parameters_.T_IMU_CAM, sensor_parameters_.fov_angle_x,
        sensor_parameters_.fov_angle_y, sensor_parameters_.coefficient_A,
        sensor_parameters_.coefficient_B,esdf_map_,
        map_parameters_.width,map_parameters_.height,
        planning_parameters_.minimum_height,planning_parameters_.maximum_height,
        optimization_parameters_.cmaes_objective,if_kfupdate);
    control_poses_.push_back(best_pose);
    previous_pose = best_pose;
    if(optimization_parameters_.cmaes_objective=="covariance")
      LOG(INFO) << "Trace of P: " << simulated_grid_map.computeCovarianceTrace();
    else if(optimization_parameters_.cmaes_objective=="acquisition_EI" || optimization_parameters_.cmaes_objective=="acquisition_PI"
         || optimization_parameters_.cmaes_objective == "acquisition_UCB")
      LOG(INFO) << "Acquisition view : " << simulated_grid_map.getAcquisitionView();
  }
    return true;
}

geometry_msgs::Pose tmPlanner::findNextBestGridPoint(
    const grid_map::GridMap& simulated_grid_map,
    const geometry_msgs::Pose& previous_pose) {
  geometry_msgs::Pose evaluated_pose;
  evaluated_pose.orientation =
      tf::createQuaternionMsgFromYaw(planning_parameters_.reference_yaw);

  double objective_prev;
  if(optimization_parameters_.cmaes_objective=="covariance")
  {
    if (planning_parameters_.use_threshold) {
      objective_prev = simulated_grid_map.computeCovarianceTrace(
          planning_parameters_.lower_threshold);
    } else {
      objective_prev = simulated_grid_map.computeCovarianceTrace();
    }
  }
  else if(optimization_parameters_.cmaes_objective=="acquisition_EI" || optimization_parameters_.cmaes_objective=="acquisition_PI"
       || optimization_parameters_.cmaes_objective == "acquisition_UCB")
  {
    // objective_prev = simulated_grid_map.getAcquisitionSum();
    // objective_prev = simulated_grid_map.getAcquisitionMax();

  }

  // Initialize the best solution found so far.
  double maximum_objective = 0.0;
  geometry_msgs::Pose best_pose;

  // Simulate measurement for candidate points in lattice and
  // calculate the informative objective.
  for (const auto& evaluated_point : lattice_.getLatticePoints()) {
    evaluated_pose.position = evaluated_point;
    // Create a copy of the grid map for evaluating each lattice point.
    grid_map::GridMap evaluated_grid_map = simulated_grid_map;
    bool if_kfupdate = true;
    evaluated_grid_map.predictMapUpdate(
        evaluated_pose, sensor_parameters_.T_IMU_CAM,
        sensor_parameters_.fov_angle_x, sensor_parameters_.fov_angle_y,
        sensor_parameters_.coefficient_A, sensor_parameters_.coefficient_B,esdf_map_,
        map_parameters_.width,map_parameters_.height,
        planning_parameters_.minimum_height,planning_parameters_.maximum_height,
        optimization_parameters_.cmaes_objective,if_kfupdate);

    // Calculate gain (reduction in objective).
    double objective_final,gain;
    if(optimization_parameters_.cmaes_objective=="covariance")
    {
      if (planning_parameters_.use_threshold) {
        objective_final = evaluated_grid_map.computeCovarianceTrace(
            planning_parameters_.lower_threshold);
      } else {
        objective_final = evaluated_grid_map.computeCovarianceTrace();
      }
      gain = objective_prev - objective_final;
    }
    else if(optimization_parameters_.cmaes_objective=="acquisition_EI" || optimization_parameters_.cmaes_objective=="acquisition_PI"
         || optimization_parameters_.cmaes_objective == "acquisition_UCB")
    {
      // objective_final = evaluated_grid_map.getAcquisitionSum();
      // objective_final = evaluated_grid_map.getAcquisitionMax();
      gain = evaluated_grid_map.getAcquisitionView();
    }

    // Calculate cost (time from previous viewpoint), assuming constant
    // speed. Limit based on measurement frequency.
    auto cost = std::max(getDistanceBetweenPoints(previous_pose.position,
                                                  evaluated_pose.position) /
                             planning_parameters_.reference_speed,
                         1.0 / sensor_parameters_.measurement_frequency);
    // Calculate the objective: rate of information gain.
    double objective = gain / cost;

    if (objective > maximum_objective && isObstacleFreePath(previous_pose.position,evaluated_pose.position)) 
    {
      maximum_objective = objective;
      best_pose = evaluated_pose;
    }
  }
  LOG(INFO) << "Best position: " << best_pose.position;
  LOG(INFO) << "Maximum objective: " << maximum_objective;
  // Return out-of-bounds value if planning is complete.
  if (maximum_objective <= 0.0) {
    LOG(INFO) << "Could not find the best lattice point! Planning is complete.";
    best_pose.position.x = 1000.0;
    best_pose.position.y = 1000.0;
    best_pose.position.z = -1.0;
  }
  return best_pose;
}

geometry_msgs::Pose tmPlanner::findNextBestGridPointRandom(
    const grid_map::GridMap& simulated_grid_map,
    const geometry_msgs::Pose& previous_pose) {
  geometry_msgs::Pose evaluated_pose;
  evaluated_pose.orientation =
      tf::createQuaternionMsgFromYaw(planning_parameters_.reference_yaw);

  double objective_prev;
  if(optimization_parameters_.cmaes_objective=="covariance")
  {
    if (planning_parameters_.use_threshold) {
      objective_prev = simulated_grid_map.computeCovarianceTrace(
          planning_parameters_.lower_threshold);
    } else {
      objective_prev = simulated_grid_map.computeCovarianceTrace();
    }
  }
  else if(optimization_parameters_.cmaes_objective=="acquisition_EI" || optimization_parameters_.cmaes_objective=="acquisition_PI"
       || optimization_parameters_.cmaes_objective == "acquisition_UCB")
  {
    // objective_prev = simulated_grid_map.getAcquisitionMax();
    // objective_prev = simulated_grid_map.getAcquisitionSum();
  }
  else
    LOG(INFO)<<"Cmaes objective type not declared correctly";

  // Initialize the best solution found so far.
  double maximum_objective = 0.0;
  geometry_msgs::Pose best_pose;

  // Simulate measurement for candidate points in lattice and
  // calculate the informative objective.

  // voxblox::Point min_bound_ = simulation_world_.getMinBound();
  // voxblox::Point max_bound_ = simulation_world_.getMaxBound();
  voxblox::Point min_bound_(-map_parameters_.width/2,-map_parameters_.height/2,planning_parameters_.minimum_height);
  voxblox::Point max_bound_(map_parameters_.width/2,map_parameters_.height/2,planning_parameters_.maximum_height);
  auto random_choice = "layer";  // "full" 3D random points or "layer" of random points  
  
  for (int i = 0; i<20; i++) 
  {
    // Random point from the entire 3D space that is in the current visibility 
    if(random_choice == "full")
    {
        do
        {
          evaluated_pose.position.x = min_bound_[0] + static_cast <float> (rand()) / \
                    ( static_cast <float> (RAND_MAX/(max_bound_[0]-min_bound_[0])));
          evaluated_pose.position.y = min_bound_[1] + static_cast <float> (rand()) / \
                    ( static_cast <float> (RAND_MAX/(max_bound_[1]-min_bound_[1])));
          evaluated_pose.position.z = min_bound_[2] + static_cast <float> (rand()) / \
                    ( static_cast <float> (RAND_MAX/(max_bound_[2]-min_bound_[2])));
        }while(!isObstacleFreePath(previous_pose.position,evaluated_pose.position));
    }
    // Random point from 4 planes/layers that are in the current visibility
    else if(random_choice == "layer")
    {
      int no_layers = 4;
      double top_clearance = 2;
      do
      {
        evaluated_pose.position.x = min_bound_[0] + static_cast <float> (rand()) / \
                  ( static_cast <float> (RAND_MAX/(max_bound_[0]-min_bound_[0])));
        evaluated_pose.position.y = min_bound_[1] + static_cast <float> (rand()) / \
                  ( static_cast <float> (RAND_MAX/(max_bound_[1]-min_bound_[1])));
        // Also include 1 metre clearance from the top for good initial solution
        evaluated_pose.position.z = min_bound_[2] + ((max_bound_[2]-min_bound_[2]-top_clearance)*((i%no_layers)+1))/no_layers;
      }while(!isObstacleFreePath(previous_pose.position,evaluated_pose.position));
    }
    else if(random_choice == "top_layer")
    {
      int no_layers = 1;
      double top_clearance = 2;
      do
      {
        evaluated_pose.position.x = min_bound_[0] + static_cast <float> (rand()) / \
                  ( static_cast <float> (RAND_MAX/(max_bound_[0]-min_bound_[0])));
        evaluated_pose.position.y = min_bound_[1] + static_cast <float> (rand()) / \
                  ( static_cast <float> (RAND_MAX/(max_bound_[1]-min_bound_[1])));
        // Also include 1 metre clearance from the top for good initial solution
        evaluated_pose.position.z = min_bound_[2] + ((max_bound_[2]-min_bound_[2]-top_clearance)*((i%no_layers)+1))/no_layers;
      }while(!isObstacleFreePath(previous_pose.position,evaluated_pose.position));
    }
    //LOG(INFO)<<"Sampled point: ("<<evaluated_pose.position.x<<","<<\
              evaluated_pose.position.y<<","<<evaluated_pose.position.z<<")";

    // Create a copy of the grid map for evaluating each lattice point.
    grid_map::GridMap evaluated_grid_map = simulated_grid_map;
    bool if_kfupdate = true;
    evaluated_grid_map.predictMapUpdate(
        evaluated_pose, sensor_parameters_.T_IMU_CAM,
        sensor_parameters_.fov_angle_x, sensor_parameters_.fov_angle_y,
        sensor_parameters_.coefficient_A, sensor_parameters_.coefficient_B,esdf_map_,
        map_parameters_.width,map_parameters_.height,
        planning_parameters_.minimum_height,planning_parameters_.maximum_height,
        optimization_parameters_.cmaes_objective,if_kfupdate);

    // Calculate gain (reduction in objective).
    double objective_final,gain;
    if(optimization_parameters_.cmaes_objective=="covariance")
    {
      if (planning_parameters_.use_threshold) {
        objective_final = evaluated_grid_map.computeCovarianceTrace(
            planning_parameters_.lower_threshold);
      } else {
        objective_final = evaluated_grid_map.computeCovarianceTrace();
      }
      gain = objective_prev - objective_final;
    }
    else if(optimization_parameters_.cmaes_objective=="acquisition_EI" || optimization_parameters_.cmaes_objective=="acquisition_PI"
         || optimization_parameters_.cmaes_objective == "acquisition_UCB")
    {
      // objective_final = evaluated_grid_map.getAcquisitionSum();
      //objective_final =evaluated_grid_map.getAcquisitionMax();
      gain = evaluated_grid_map.getAcquisitionView();
    }

    
    // Calculate cost (time from previous viewpoint), assuming constant
    // speed. Limit based on measurement frequency.
    auto cost = std::max(getDistanceBetweenPoints(previous_pose.position,
                                                  evaluated_pose.position) /
                             planning_parameters_.reference_speed,
                         1.0 / sensor_parameters_.measurement_frequency);
    // Calculate the objective: rate of information gain.
    double objective = gain / cost;
    // LOG(INFO) << "Random selected objective: "<<objective;
    // LOG(INFO) << "Random selected gain: "<<gain;

    if (objective > maximum_objective) 
    {
      maximum_objective = objective;
      best_pose = evaluated_pose;
    }
  }
  LOG(INFO) << "Best position: " << best_pose.position;
  LOG(INFO) << "Maximum objective: " << maximum_objective;
  // Return out-of-bounds value if planning is complete.
  if (maximum_objective <= 0.0) {
    LOG(INFO) << "Could not find the best lattice point! Planning is complete.";
    best_pose.position.x = 1000.0;
    best_pose.position.y = 1000.0;
    best_pose.position.z = -1.0;
  }
  return best_pose;
}

bool tmPlanner::isObstacleFreePath(const geometry_msgs::Point& previous_pose,const geometry_msgs::Point& evaluated_point)
{
  //LOG(INFO) << "Debug me now please:  isObstacleFreePath() ";
  bool if_collision = false;
  double norm_dist, dist_incr = 0.05, dist_curr;

  //Check if destination is close to obstacles
  Eigen::Vector3d evaluated_point_copy; // to deal with data type bullshit
  evaluated_point_copy[0] = evaluated_point.x;
  evaluated_point_copy[1] = evaluated_point.y;
  evaluated_point_copy[2] = evaluated_point.z;
  double dist_evaluated_point;
  if(esdf_map_->getDistanceAtPosition(evaluated_point_copy,&dist_evaluated_point))
    {
      if(dist_evaluated_point<=0.75)   // To prevent the control points from being close to obstacles
        return false;   
    }
  else
  {
    LOG(ERROR)<<"The destination ("<<evaluated_point.x<<","<<evaluated_point.y<<","<<evaluated_point.z \
        <<" is outside the map! Inside isObstacleFreePath()";
    return false; 
  }

  // Check if UAV is close to obstacles already
  Eigen::Vector3d previous_pose_copy; // to deal with data type bullshit
  previous_pose_copy[0] = previous_pose.x;
  previous_pose_copy[1] = previous_pose.y;
  previous_pose_copy[2] = previous_pose.z;
  if(esdf_map_->getDistanceAtPosition(previous_pose_copy,&dist_curr))
    {
      if(dist_curr<=0.75)
        dist_incr = 0.75;    // Throw the first point out of the obstacle
    }
  else
  {
    LOG(ERROR)<<"The current position ("<<previous_pose.x<<","<<previous_pose.y<<","<<previous_pose.z \
        <<" is outside the map! Inside isObstacleFreePath()";
    return true; // Worst case scenario here, just save the drone somehow by getting inside the boundary!
  }

  norm_dist = getDistanceBetweenPoints(previous_pose,evaluated_point);
  double u = dist_incr/norm_dist;
  while(u<1)
  {
    Eigen::Vector3d in_point;
    in_point[0] = (1-u)*previous_pose.x + u*evaluated_point.x;
    in_point[1] = (1-u)*previous_pose.y + u*evaluated_point.y;
    in_point[2] = (1-u)*previous_pose.z + u*evaluated_point.z;
    double dist;
    if(esdf_map_->getDistanceAtPosition(in_point,&dist))
    {
      if(dist<=0.75)
        return false;
    }
    else
    {
      LOG(ERROR)<<"The point ("<<in_point[0]<<","<<in_point[1]<<","<<in_point[2] \
        <<") doesn't exist in the map or is not observed! Inside isObstacleFreePath()";
        return false; // sleepy, so check it later
    }
    u = u + (dist/norm_dist);
    //LOG(INFO)<<" Dist: "<<dist;
  }

  return true;
}

void tmPlanner::optimizePathOnHorizon() {

  // Use the CMA-ES.
  if (optimization_parameters_.optimization_method == "cmaes") {

    CHECK_GT(control_poses_.size(), 0) << "No control poses to optimize!";

    // Initialize optimization parameters.
    control_poses_.pop_front();
    int dim = control_poses_.size() * kDimensions;
    std::vector<double> x0;
    for (auto pose : control_poses_) {
      x0.push_back(pose.position.x);
      x0.push_back(pose.position.y);
      x0.push_back(pose.position.z);
    }

    double lbounds[dim], ubounds[dim];
    for (size_t i = 0; i < dim; i += 3) {
      lbounds[i] =
          -(grid_map_.getLength()(0) * grid_map_.getResolution()(0)) / 2.0;
      ubounds[i] =
          grid_map_.getLength()(0) * grid_map_.getResolution()(0) / 2.0;
      lbounds[i + 1] =
          -(grid_map_.getLength()(1) * grid_map_.getResolution()(1)) / 2.0;
      ubounds[i + 1] =
          grid_map_.getLength()(1) * grid_map_.getResolution()(1) / 2.0;
      lbounds[i + 2] = planning_parameters_.minimum_height;
      ubounds[i + 2] = planning_parameters_.maximum_height;
    }

    // Perform the optimization.
    libcmaes::FitFunc optimize_control_poses =
        std::bind(&tmPlanner::optimizeControlPoses, this,
                  std::placeholders::_1, std::placeholders::_2);
    libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> geno(lbounds, ubounds, dim);
    libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>>
        cmaparams(dim, &x0.front(), optimization_parameters_.cmaes_step_size,
                  optimization_parameters_.cmaes_offsprings, 0, geno);
    cmaparams.set_max_fevals(optimization_parameters_.cmaes_maximum_fevals);


    // cmaparams.set_algo(aCMAES); // active CMA-ES

    // For BIPOP
    int  kk = aBIPOP_CMAES; // aCMAES
    cmaparams.set_algo(kk);
    cmaparams.set_restarts(1);

    cmaparams.set_fplot(ros::package::getPath("tmplanner_continuous") + "/debug/matrices/cmaes_results" + ".dat");
    libcmaes::CMASolutions cmasols =
        libcmaes::cmaes<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>>(
            optimize_control_poses, cmaparams);

    // Write best solution.
    const Eigen::VectorXd x_best =
        geno.pheno(cmasols.get_best_seen_candidate().get_x_dvec());
    control_poses_.clear();
    // Compute world to vicon-sensor-body transform for trajectory commands.
    kindr::minimal::QuatTransformation T_MAP_VSB;
    tf::poseMsgToKindr(odometry_pose_, &T_MAP_VSB);
    kindr::minimal::QuatTransformation T_W_VSB =
        map_parameters_.T_W_MAP * T_MAP_VSB;
    geometry_msgs::Pose pose_world;
    tf::poseKindrToMsg(T_W_VSB, &pose_world);
    control_poses_.push_back(pose_world);
    geometry_msgs::Pose pose;
    for (size_t i = 0; i < dim; i += 3) {
      pose.position.x = x_best[i];
      pose.position.y = x_best[i + 1];
      pose.position.z = x_best[i + 2];
      pose.orientation =
          tf::createQuaternionMsgFromYaw(planning_parameters_.reference_yaw);
      tf::poseMsgToKindr(pose, &T_MAP_VSB);
      T_W_VSB = map_parameters_.T_W_MAP * T_MAP_VSB;
      tf::poseKindrToMsg(T_W_VSB, &pose_world);
      control_poses_.push_back(pose_world);
    }
  }
  else if(optimization_parameters_.optimization_method == "bayesian_opt")
  {

    int n = (planning_parameters_.control_poses-1) * kDimensions;                   // Number of dimensions

    // Common configuration
    // See parameters.h for the available options.
    // Some parameters did not need to be changed for default, only for
    // illustrative purpose.
    bayesopt::Parameters par = initialize_parameters_to_default();

    par.kernel.name = "kSum(kSEISO,kConst)";
    par.kernel.hp_mean <<= 1.0, 1.0;
    par.kernel.hp_std <<= 1.0, 1.0;

    par.mean.name = "mConst";
    par.mean.coef_mean <<= 1.0;
    par.mean.coef_std <<= 1.0;
    

    par.surr_name = "sStudentTProcessJef";
    par.noise = 1e-10;

    par.sc_type = SC_MAP;
    par.l_type = L_EMPIRICAL;

    par.n_iterations = 15;    // Number of iterations
    par.random_seed = -1;
    par.n_init_samples = 19;
    par.n_iter_relearn = 5;

    bayesianOpt opt(n,par,this);
    vectord x_best(n);

    //Define bounds and prepare result.
    boost::numeric::ublas::vector<double> lowerBound(n);
    boost::numeric::ublas::vector<double> upperBound(n);

    for(int i = 0; i< n; i += 3)
    {
      lowerBound(i) = -map_parameters_.width/2;
      lowerBound(i+1) = -map_parameters_.height/2;
      lowerBound(i+2) = planning_parameters_.minimum_height;

      upperBound(i) = map_parameters_.width/2;
      upperBound(i+1) = map_parameters_.height/2;
      upperBound(i+2) = planning_parameters_.maximum_height;
    }

    //Set the bounds. This is optional. Default is [0,1]
    //Only required because we are doing continuous optimization
    opt.setBoundingBox(lowerBound,upperBound);

    // Run C++ interface
    opt.optimize(x_best);

    // Results
    std::cout << "Final result: " << x_best(0) <<" first "<<x_best<< std::endl;

    control_poses_.clear();
    // Compute world to vicon-sensor-body transform for trajectory commands.
    kindr::minimal::QuatTransformation T_MAP_VSB;
    tf::poseMsgToKindr(odometry_pose_, &T_MAP_VSB);
    kindr::minimal::QuatTransformation T_W_VSB =
        map_parameters_.T_W_MAP * T_MAP_VSB;
    geometry_msgs::Pose pose_world;
    tf::poseKindrToMsg(T_W_VSB, &pose_world);
    control_poses_.push_back(pose_world);
    geometry_msgs::Pose pose;
    for (size_t i = 0; i < n; i += 3) {
      pose.position.x = x_best(i);
      pose.position.y = x_best(i + 1);
      pose.position.z = x_best(i + 2);
      pose.orientation =
          tf::createQuaternionMsgFromYaw(planning_parameters_.reference_yaw);
      tf::poseMsgToKindr(pose, &T_MAP_VSB);
      T_W_VSB = map_parameters_.T_W_MAP * T_MAP_VSB;
      tf::poseKindrToMsg(T_W_VSB, &pose_world);
      control_poses_.push_back(pose_world);
    }
  }
}

/* Function to be used for C-API testing */
double tmPlanner::testFunction(unsigned int n, const double *x,
        double *gradient, /* NULL if not needed */
        void *func_data)
{
  double f = 10.;
  for (unsigned int i = 0; i < n; ++i)
    {
      f += (x[i] - .53) * (x[i] - .53);
    }
  return f;
}

double tmPlanner::bayesianOptimizeControlPoses( const vectord& x){
  // Add current pose as first control pose for the polynomial.
  control_poses_.clear();
  control_poses_.push_back(odometry_pose_);

  //double obj = 0;
  // Construct the list of control poses.
  for (size_t i = 0; i < (planning_parameters_.control_poses-1) * kDimensions; i += 3) {
    geometry_msgs::Pose pose;
    pose.position.x = x(i);
    pose.position.y = x(i + 1);
    pose.position.z = x(i + 2);
    pose.orientation =
        tf::createQuaternionMsgFromYaw(planning_parameters_.reference_yaw);
    control_poses_.push_back(pose);
    //obj = obj + sqrt(pow(x[i],2)+pow(x[i+1],2)+pow(x[i+2],2));
  }

  auto gain = 0.0;
  auto simulated_grid_map = grid_map_;
  double objective_prev;
  if(optimization_parameters_.cmaes_objective=="covariance")
  {
    if (planning_parameters_.use_threshold) {
      objective_prev = simulated_grid_map.computeCovarianceTrace(
          planning_parameters_.lower_threshold);
    } else {
      objective_prev = simulated_grid_map.computeCovarianceTrace();
    }
  }
  else if(optimization_parameters_.cmaes_objective=="acquisition_EI" || optimization_parameters_.cmaes_objective=="acquisition_PI"
       || optimization_parameters_.cmaes_objective == "acquisition_UCB")
  {
    // objective_prev = simulated_grid_map.getAcquisitionSum();
    // objective_prev = simulated_grid_map.getAcquisitionMax();
  }


  createPolynomialTrajectory();
  mav_msgs::EigenTrajectoryPoint::Vector measurement_states;
  mav_trajectory_generation::sampleWholeTrajectory(
      trajectory_, 1.0 / sensor_parameters_.measurement_frequency,
      &measurement_states);
  // LOG(INFO)<<"Number of trajectory points: "<<measurement_states.size();

  // Discard paths that are too long.
  // if (measurement_states.size() > 10) {
  //   return 1001;
  // }

  // Perform map update predictions.
  for (const auto& measurement_state : measurement_states) {
    geometry_msgs::PoseStamped measurement_pose;
    mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(measurement_state,
                                                     &measurement_pose);

    
    bool if_kfupdate = true;
    simulated_grid_map.predictMapUpdate(
        measurement_pose.pose, sensor_parameters_.T_IMU_CAM,
        sensor_parameters_.fov_angle_x, sensor_parameters_.fov_angle_y,
        sensor_parameters_.coefficient_A, sensor_parameters_.coefficient_B,esdf_map_,
        map_parameters_.width,map_parameters_.height,
        planning_parameters_.minimum_height,planning_parameters_.maximum_height,
        optimization_parameters_.cmaes_objective,if_kfupdate);

    if(optimization_parameters_.cmaes_objective=="acquisition_EI" || optimization_parameters_.cmaes_objective=="acquisition_PI"
         || optimization_parameters_.cmaes_objective == "acquisition_UCB")
      gain = gain + simulated_grid_map.getAcquisitionView();

  }

  double objective_final;
  if(optimization_parameters_.cmaes_objective=="covariance")
  {
    if (planning_parameters_.use_threshold) {
      objective_final = simulated_grid_map.computeCovarianceTrace(
          planning_parameters_.lower_threshold);
    } else {
      objective_final = simulated_grid_map.computeCovarianceTrace();
    }
    gain = objective_prev - objective_final;
  }


  // Consider Obstacles and environment boundary into optimization cost

  double dist, dist_cost = 0.0, dist_cntl;

  
  // Consider points in the trajectory for obstacles
  mav_msgs::EigenTrajectoryPoint::Vector obst_check_points;
  float obst_check_frequency = 4;
  mav_trajectory_generation::sampleWholeTrajectory(
      trajectory_, 1.0 / obst_check_frequency, &obst_check_points);

  int i=0;
  for (const auto& obst_points : obst_check_points) 
  {
    if(esdf_map_->getDistanceAtPosition(obst_points.position_W,&dist))
    {
      dist_cost = dist_cost + obstacleCost(dist,"hard");
    }
    else
    {
      //boundary_cost = boundary_cost + 50
      LOG(ERROR)<<"The point ("<<obst_points.position_W[0]<<","<<\
        obst_points.position_W[1]<<","<<obst_points.position_W[2]\
        <<") doesn't exist in the map or is not observed - optimizeControlPoses()! ";
      // return 103;
    }
    i++;
  }
  //LOG(INFO)<<"Considering "<<i<<" points in trajectory for collision";


  auto cost = trajectory_.getMaxTime();
  gain = gain + dist_cost;
  LOG(INFO) << "Objective = " << -gain / cost;
  return -gain / cost;
  //LOG(INFO) << "Objective = " << -dist_cost;
  //return -dist_cost;
  //return obj;
}


double tmPlanner::optimizeControlPoses(const double* x, const int N) {
  // Add current pose as first control pose for the polynomial.
  control_poses_.clear();
  control_poses_.push_back(odometry_pose_);

  //double obj = 0;
  // Construct the list of control poses.
  for (size_t i = 0; i < N; i += 3) {
    geometry_msgs::Pose pose;
    pose.position.x = x[i];
    pose.position.y = x[i + 1];
    pose.position.z = x[i + 2];
    pose.orientation =
        tf::createQuaternionMsgFromYaw(planning_parameters_.reference_yaw);
    control_poses_.push_back(pose);
    //obj = obj + sqrt(pow(x[i],2)+pow(x[i+1],2)+pow(x[i+2],2));
  }

  auto gain = 0.0;
  auto simulated_grid_map = grid_map_;
  double objective_prev;
  if(optimization_parameters_.cmaes_objective=="covariance")
  {
    if (planning_parameters_.use_threshold) {
      objective_prev = simulated_grid_map.computeCovarianceTrace(
          planning_parameters_.lower_threshold);
    } else {
      objective_prev = simulated_grid_map.computeCovarianceTrace();
    }
  }
  else if(optimization_parameters_.cmaes_objective=="acquisition_EI" || optimization_parameters_.cmaes_objective=="acquisition_PI"
       || optimization_parameters_.cmaes_objective == "acquisition_UCB")
  {
    // objective_prev = simulated_grid_map.getAcquisitionSum();
    // objective_prev = simulated_grid_map.getAcquisitionMax();
  }


  createPolynomialTrajectory();
  mav_msgs::EigenTrajectoryPoint::Vector measurement_states;
  mav_trajectory_generation::sampleWholeTrajectory(
      trajectory_, 1.0 / sensor_parameters_.measurement_frequency,
      &measurement_states);
  // LOG(INFO)<<"Number of trajectory points: "<<measurement_states.size();

  // Discard paths that are too long.
  // if (measurement_states.size() > 10) {
  //   return 1001;
  // }

  // Perform map update predictions.
  for (const auto& measurement_state : measurement_states) {
    geometry_msgs::PoseStamped measurement_pose;
    mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(measurement_state,
                                                     &measurement_pose);
    // Discard measurement locations that are out-of-bounds.
    // if ((measurement_pose.pose.position.x <
    //      -(grid_map_.getLength()(0) * grid_map_.getResolution()(0)) / 2.0) ||
    //     (measurement_pose.pose.position.x >
    //      (grid_map_.getLength()(0) * grid_map_.getResolution()(0)) / 2.0) ||
    //     (measurement_pose.pose.position.y <
    //      -(grid_map_.getLength()(1) * grid_map_.getResolution()(1)) / 2.0) ||
    //     (measurement_pose.pose.position.y <
    //      -(grid_map_.getLength()(1) * grid_map_.getResolution()(1)) / 2.0) ||
    //     measurement_pose.pose.position.z <
    //         planning_parameters_.minimum_height ||
    //     measurement_pose.pose.position.z >
    //         planning_parameters_.maximum_height) {
    //   LOG(ERROR)<<"Out of bounds control points in optimizeControlPoses()";
    //   //return 101;
    // }
    
    bool if_kfupdate = true;
    simulated_grid_map.predictMapUpdate(
        measurement_pose.pose, sensor_parameters_.T_IMU_CAM,
        sensor_parameters_.fov_angle_x, sensor_parameters_.fov_angle_y,
        sensor_parameters_.coefficient_A, sensor_parameters_.coefficient_B,esdf_map_,
        map_parameters_.width,map_parameters_.height,
        planning_parameters_.minimum_height,planning_parameters_.maximum_height,
        optimization_parameters_.cmaes_objective,if_kfupdate);

    if(optimization_parameters_.cmaes_objective=="acquisition_EI" || optimization_parameters_.cmaes_objective=="acquisition_PI"
         || optimization_parameters_.cmaes_objective == "acquisition_UCB")
      gain = gain + simulated_grid_map.getAcquisitionView();

  }

  double objective_final;
  if(optimization_parameters_.cmaes_objective=="covariance")
  {
    if (planning_parameters_.use_threshold) {
      objective_final = simulated_grid_map.computeCovarianceTrace(
          planning_parameters_.lower_threshold);
    } else {
      objective_final = simulated_grid_map.computeCovarianceTrace();
    }
    gain = objective_prev - objective_final;
  }
  // else if(optimization_parameters_.cmaes_objective=="acquisition")
  // {
  //   // objective_final = simulated_grid_map.getAcquisitionSum();
  //   // objective_final = simulated_grid_map.getAcquisitionMax();
  //   gain = simulated_grid_map.getAcquisitionView();
  // }

  // Consider Obstacles and environment boundary into optimization cost

  double dist, dist_cost = 0.0, dist_cntl;

  // Consider control poses that are close to obstacles
  // for (const auto& control_poses_obs : control_poses_) 
  // {
  //   Eigen::Vector3d control_poses_obs_copy;   // data type bullshit again
  //   control_poses_obs_copy[0] = control_poses_obs.position.x;
  //   control_poses_obs_copy[1] = control_poses_obs.position.y;
  //   control_poses_obs_copy[2] = control_poses_obs.position.z;

  //   if(esdf_map_->getDistanceAtPosition(control_poses_obs_copy,&dist_cntl))
  //   {
  //     if(dist_cntl<0.5)
  //     {
  //       dist_cost = dist_cost + 50;
  //       LOG(ERROR)<<"Control point collides! optimizeControlPoses()";
  //     }
  //   }
  //   else
  //   {
  //     //boundary_cost = boundary_cost + 50
  //     LOG(ERROR)<<"The control point ("<<control_poses_obs_copy[0]<<","<<\
  //       control_poses_obs_copy[1]<<","<<control_poses_obs_copy[2]\
  //       <<") doesn't exist in the map or is not observed - optimizeControlPoses()! ";
  //     //return 102;
  //   }
  // }
  
  // Consider points in the trajectory for obstacles
  mav_msgs::EigenTrajectoryPoint::Vector obst_check_points;
  float obst_check_frequency = 4;
  mav_trajectory_generation::sampleWholeTrajectory(
      trajectory_, 1.0 / obst_check_frequency, &obst_check_points);

  int i=0;
  for (const auto& obst_points : obst_check_points) 
  {
    if(esdf_map_->getDistanceAtPosition(obst_points.position_W,&dist))
    {
      dist_cost = dist_cost + obstacleCost(dist,"hard");
    }
    else
    {
      //boundary_cost = boundary_cost + 50
      LOG(ERROR)<<"The point ("<<obst_points.position_W[0]<<","<<\
        obst_points.position_W[1]<<","<<obst_points.position_W[2]\
        <<") doesn't exist in the map or is not observed - optimizeControlPoses()! ";
      //return 103;
    }
    i++;
  }
  //LOG(INFO)<<"Considering "<<i<<" points in trajectory for collision";


  auto cost = trajectory_.getMaxTime();
  gain = gain + dist_cost;
  LOG(INFO) << "Objective = " << -gain / cost;
  return -gain / cost;
  //LOG(INFO) << "Objective = " << -dist_cost;
  //return -dist_cost;
  //return obj;
}


double tmPlanner::obstacleCost(double dist, std::string cost_type)
{
  if(boost::iequals(cost_type,"sum"))
    return dist;
  else if(boost::iequals(cost_type,"sigmoid"))
    return 1/(1+exp(-dist));
  else if(boost::iequals(cost_type,"logistic"))
  {
    if(dist<=0)
      return -25+20*dist; 
    else
      return (25/(1+exp(-10*(dist-0.5))))-25;
  }
  else if(boost::iequals(cost_type,"hard"))
  {
    if(dist>0.5)
      return 0;
    else
    {
      //LOG(INFO) << "Colliding path  ";
      return -25;
    }
  }
  else
  {
    return -1000; //testing
    LOG(ERROR)<<"Cost function for obstacle avoidance not available";
  }
}

bool tmPlanner::arePointsEqual(const geometry_msgs::Point& p1,
                                 const geometry_msgs::Point& p2,
                                 const double& tol) {
  return (fabs(p1.x - p2.x) < tol) && (fabs(p1.y - p2.y) < tol) &&
         (fabs(p1.z - p2.z) < tol);
}

double tmPlanner::getDistanceBetweenPoints(const geometry_msgs::Point& p1,
                                             const geometry_msgs::Point& p2) {
  return sqrt(pow(p1.x - p2.x, 2.0) + pow(p1.y - p2.y, 2.0) +
              pow(p1.z - p2.z, 2.0));
}