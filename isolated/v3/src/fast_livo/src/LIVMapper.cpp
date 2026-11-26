#include <atomic>
#include <chrono>
#include <cmath>
namespace { std::atomic<unsigned long long> eval_lidar_rx{0}, eval_imu_rx{0}, eval_image_rx{0}; }
/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include "rolling_csv.hpp"
#include "startup_parameters.h"
#include <vikit/camera_loader.h>

using namespace Sophus;
LIVMapper::LIVMapper(rclcpp::Node::SharedPtr &node, std::string node_name)
    : node(std::make_shared<rclcpp::Node>(node_name)),
      extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);
  
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(this->node);
  VoxelMapConfig voxel_config;
  loadVoxelConfig(this->node, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents(this->node);          // initialize components errors
  path.header.stamp = this->node->now();
  path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters(rclcpp::Node::SharedPtr &node)
{
  // declare parameters
  // Frozen pinhole calibration: declare before Vikit reads ROS overrides.
  this->node->declare_parameter<std::string>("parameter_blackboard.model");
  this->node->declare_parameter<int>("parameter_blackboard.width");
  this->node->declare_parameter<int>("parameter_blackboard.height");
  this->node->declare_parameter<double>("parameter_blackboard.scale");
  this->node->declare_parameter<double>("parameter_blackboard.fx");
  this->node->declare_parameter<double>("parameter_blackboard.fy");
  this->node->declare_parameter<double>("parameter_blackboard.cx");
  this->node->declare_parameter<double>("parameter_blackboard.cy");
  this->node->declare_parameter<double>("parameter_blackboard.d0");
  this->node->declare_parameter<double>("parameter_blackboard.d1");
  this->node->declare_parameter<double>("parameter_blackboard.d2");
  this->node->declare_parameter<double>("parameter_blackboard.d3");
  this->node->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
  this->node->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
  this->node->declare_parameter<bool>("common.ros_driver_bug_fix", false);
  this->node->declare_parameter<int>("common.img_en", 1);
  this->node->declare_parameter<int>("common.lidar_en", 1);
  this->node->declare_parameter<std::string>("common.img_topic", "/left_camera/image");

  this->node->declare_parameter<bool>("vio.normal_en", true);
  this->node->declare_parameter<bool>("vio.inverse_composition_en", false);
  this->node->declare_parameter<int>("vio.max_iterations", 5);
  this->node->declare_parameter<int>("vio.img_point_cov", 100);
  this->node->declare_parameter<bool>("vio.raycast_en", false);
  exposure_estimate_en = startup_parameters::declareWithAlias(
      *node, "vio.exposure_estimate_en", "imu.exposure_estimate_en", true);
  this->node->declare_parameter<double>("vio.inv_expo_cov", 0.1);
  this->node->declare_parameter<int>("vio.grid_size", 5);
  this->node->declare_parameter<int>("vio.grid_n_height", 17);
  this->node->declare_parameter<int>("vio.patch_pyrimid_level", 4);
  this->node->declare_parameter<int>("vio.patch_size", 8);
  this->node->declare_parameter<int>("vio.outlier_threshold", 100);
  this->node->declare_parameter<double>("time_offset.exposure_time_init", 0.0);
  this->node->declare_parameter<double>("time_offset.img_time_offset", 0.0);
  this->node->declare_parameter<double>("time_offset.imu_time_offset", 0.0);
  this->node->declare_parameter<double>("time_offset.lidar_time_offset", 0.0);
  this->node->declare_parameter<bool>("uav.imu_rate_odom", false);
  this->node->declare_parameter<bool>("uav.gravity_align_en", false);

  this->node->declare_parameter<std::string>("evo.seq_name", "01");
  this->node->declare_parameter<bool>("evo.pose_output_en", false);
  this->node->declare_parameter<double>("imu.gyr_cov", 1.0);
  this->node->declare_parameter<double>("imu.acc_cov", 1.0);
  const auto fixed = startup_parameters::readOnly();
  b_gyr_cov = node->declare_parameter<double>("imu.b_gyr_cov", 0.0001, fixed);
  b_acc_cov = node->declare_parameter<double>("imu.b_acc_cov", 0.0001, fixed);
  if (!std::isfinite(b_gyr_cov) || b_gyr_cov < 0 || !std::isfinite(b_acc_cov) || b_acc_cov < 0)
    throw std::invalid_argument("imu.b_gyr_cov and imu.b_acc_cov must be finite nonnegative covariances");
  this->node->declare_parameter<int>("imu.imu_int_frame", 30);
  this->node->declare_parameter<bool>("imu.imu_en", true);
  this->node->declare_parameter<bool>("imu.gravity_est_en", true);
  this->node->declare_parameter<bool>("imu.ba_bg_est_en", true);

  this->node->declare_parameter<double>("preprocess.blind", 0.01);
    this->node->declare_parameter<bool>("preprocess.hilti_en", false);
  this->node->declare_parameter<double>("preprocess.filter_size_surf", 0.5);
  this->node->declare_parameter<int>("preprocess.lidar_type", AVIA);
  this->node->declare_parameter<int>("preprocess.scan_line",6);
  this->node->declare_parameter<int>("preprocess.point_filter_num", 3);
  this->node->declare_parameter<bool>("preprocess.feature_extract_enabled", false);

  this->node->declare_parameter<int>("pcd_save.interval", -1);
  this->node->declare_parameter<bool>("pcd_save.pcd_save_en", false);
  this->node->declare_parameter<bool>("image_save.img_save_en", false);
  this->node->declare_parameter<int>("image_save.interval", 1);

  this->node->declare_parameter<int>("pcd_save.type", 0);
  this->node->declare_parameter<bool>("pcd_save.colmap_output_en", false);
  this->node->declare_parameter<double>("pcd_save.filter_size_pcd", 0.5);
  this->node->declare_parameter<vector<double>>("extrin_calib.extrinsic_T", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.extrinsic_R", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.Pcl", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.Rcl", vector<double>{});
  this->node->declare_parameter<double>("debug.plot_time", -10);
  this->node->declare_parameter<int>("debug.frame_cnt", 6);

  this->node->declare_parameter<double>("publish.blind_rgb_points", 0.01);
  this->node->declare_parameter<int>("publish.pub_scan_num", 1);
  this->node->declare_parameter<bool>("publish.pub_effect_point_en", false);
  this->node->declare_parameter<bool>("publish.dense_map_en", false);

  // get parameter
  this->node->get_parameter("common.lid_topic", lid_topic);
  this->node->get_parameter("common.imu_topic", imu_topic);
  this->node->get_parameter("common.ros_driver_bug_fix", ros_driver_fix_en);
  this->node->get_parameter("common.img_en", img_en);
  this->node->get_parameter("common.lidar_en", lidar_en);
  this->node->get_parameter("common.img_topic", img_topic);

  this->node->get_parameter("vio.normal_en", normal_en);
  this->node->get_parameter("vio.inverse_composition_en", inverse_composition_en);
  this->node->get_parameter("vio.max_iterations", max_iterations);
  this->node->get_parameter("vio.img_point_cov", IMG_POINT_COV);
  this->node->get_parameter("vio.raycast_en", raycast_en);
  this->node->get_parameter("vio.inv_expo_cov", inv_expo_cov);
  this->node->get_parameter("vio.grid_size", grid_size);
  this->node->get_parameter("vio.grid_n_height", grid_n_height);
  this->node->get_parameter("vio.patch_pyrimid_level", patch_pyrimid_level);
  this->node->get_parameter("vio.patch_size", patch_size);
  this->node->get_parameter("vio.outlier_threshold", outlier_threshold);
  this->node->get_parameter("time_offset.exposure_time_init", exposure_time_init);
  this->node->get_parameter("time_offset.img_time_offset", img_time_offset);
  this->node->get_parameter("time_offset.imu_time_offset", imu_time_offset);
  this->node->get_parameter("time_offset.lidar_time_offset", lidar_time_offset);
  this->node->get_parameter("uav.imu_rate_odom", imu_prop_enable);
  this->node->get_parameter("uav.gravity_align_en", gravity_align_en);

  this->node->get_parameter("evo.seq_name", seq_name);
  this->node->get_parameter("evo.pose_output_en", pose_output_en);
  this->node->get_parameter("imu.gyr_cov", gyr_cov);
  this->node->get_parameter("imu.acc_cov", acc_cov);
  this->node->get_parameter("imu.imu_int_frame", imu_int_frame);
  this->node->get_parameter("imu.imu_en", imu_en);
  this->node->get_parameter("imu.gravity_est_en", gravity_est_en);
  this->node->get_parameter("imu.ba_bg_est_en", ba_bg_est_en);

  this->node->get_parameter("preprocess.blind", p_pre->blind);
  this->node->get_parameter("preprocess.filter_size_surf", filter_size_surf_min);
  this->node->get_parameter("preprocess.lidar_type", p_pre->lidar_type);
  this->node->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
  this->node->get_parameter("preprocess.point_filter_num", p_pre->point_filter_num);
  this->node->get_parameter("preprocess.feature_extract_enabled", p_pre->feature_enabled);

  this->node->get_parameter("pcd_save.interval", pcd_save_interval);
  this->node->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
  this->node->get_parameter("pcd_save.colmap_output_en", colmap_output_en);
  this->node->get_parameter("pcd_save.filter_size_pcd", filter_size_pcd);
  this->node->get_parameter("extrin_calib.extrinsic_T", extrinT);
  this->node->get_parameter("extrin_calib.extrinsic_R", extrinR);
  this->node->get_parameter("extrin_calib.Pcl", cameraextrinT);
  this->node->get_parameter("extrin_calib.Rcl", cameraextrinR);
  this->node->get_parameter("debug.plot_time", plot_time);
  this->node->get_parameter("debug.frame_cnt", frame_cnt);

  this->node->get_parameter("publish.blind_rgb_points", blind_rgb_points);
  this->node->get_parameter("publish.pub_scan_num", pub_scan_num);
  this->node->get_parameter("publish.pub_effect_point_en", pub_effect_point_en);
  this->node->get_parameter("publish.dense_map_en", dense_map_en);
}

void LIVMapper::initializeComponents(rclcpp::Node::SharedPtr &node) 
{
  vio_manager->lru_options.enabled = this->node->declare_parameter<bool>("vio.local_map_en", true);
  vio_manager->lru_options.radius_m = this->node->declare_parameter<double>("vio.local_map_radius_m", 30.0);
  const int lru_far = this->node->declare_parameter<int>("vio.lru_far_idle_frames", 30);
  const int lru_idle = this->node->declare_parameter<int>("vio.lru_idle_frames", 100);
  const int lru_recent = this->node->declare_parameter<int>("vio.lru_recent_frames", 10);
  if (lru_far < 1 || lru_idle < 1 || lru_recent < 1)
    throw std::invalid_argument("invalid visual map frame window");
  vio_manager->lru_options.far_idle_frames=lru_far;
  vio_manager->lru_options.idle_frames=lru_idle;
  vio_manager->lru_options.recent_frames=lru_recent;
  vio_manager->lru_options.validate();
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  
  // extrinT.assign({0.04165, 0.02326, -0.0284});
  // extrinR.assign({1, 0, 0, 0, 1, 0, 0, 0, 1});
  // cameraextrinT.assign({0.0194384, 0.104689,-0.0251952});
  // cameraextrinR.assign({0.00610193,-0.999863,-0.0154172,-0.00615449,0.0153796,-0.999863,0.999962,0.00619598,-0.0060598});

  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);
  vio_manager->lidar_root_size = voxelmap_manager->config_setting_.max_voxel_size_;

  vio_manager->exposure_estimate_en = exposure_estimate_en;
  if (img_en) {
  if (!vk::camera_loader::loadFromRosNs(this->node, "parameter_blackboard", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");
  vio_manager->owned_camera.reset(vio_manager->cam);

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->initializeVIO();

  }

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D::Constant(b_gyr_cov));
  p_imu->set_acc_bias_cov(V3D::Constant(b_acc_cov));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() 
{
  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_en) fout_lidar_pos.open(std::string(ROOT_DIR) + "Log/pcd/lidar_poses.txt", std::ios::out);
  if(img_save_en) fout_visual_pos.open(std::string(ROOT_DIR) + "Log/image/image_poses.txt", std::ios::out);
}

void LIVMapper::initializeSubscribersAndPublishers(rclcpp::Node::SharedPtr &node, image_transport::ImageTransport &it_)
{
  image_transport::ImageTransport it(this->node);
  if (p_pre->lidar_type == AVIA) {
    sub_pcl = this->node->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 32, std::bind(&LIVMapper::livox_pcl_cbk, this, std::placeholders::_1));
  } else {
    sub_pcl = this->node->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, 32, std::bind(&LIVMapper::standard_pcl_cbk, this, std::placeholders::_1));
  }
  sub_imu = this->node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 2000, std::bind(&LIVMapper::imu_cbk, this, std::placeholders::_1));
  if (img_en) sub_img = this->node->create_subscription<sensor_msgs::msg::Image>(img_topic, 8, std::bind(&LIVMapper::img_cbk, this, std::placeholders::_1));
  
  pubLaserCloudFullRes = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100);
  pubNormal = this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker", 100);
  pubSubVisualMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_visual_sub_map_before", 100);
  pubLaserCloudEffect = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100);
  pubOdomAftMapped = this->node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
  pubCulvertState = this->node->create_publisher<nav_msgs::msg::Odometry>("/culvert_source/state", 5);
  pubCulvertScan = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/culvert_source/scan", 5);
  pubPath = this->node->create_publisher<nav_msgs::msg::Path>("/path", 10);
  plane_pub = this->node->create_publisher<visualization_msgs::msg::Marker>("/planner_normal", 1);
  voxel_pub = this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj", 100);
  pubLaserCloudDynRmed = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_dbg_hist", 100);
  mavros_pose_publisher = this->node->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImage = it.advertise("/rgb_img", 1);
  pubImuPropOdom = this->node->create_publisher<nav_msgs::msg::Odometry>("/LIVO2/imu_propagate", 32);
  imu_prop_timer = this->node->create_wall_timer(0.004s, std::bind(&LIVMapper::imu_prop_callback, this));
  voxelmap_manager->voxel_map_pub_= this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/planes", 32);
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Eigen::Quaterniond G_q_I0 = Eigen::Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}

void LIVMapper::handleVIO() 
{
    
  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr)) 
  {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }
    
  RCLCPP_DEBUG(this->node->get_logger(), "Raw visual features %zu", pcl_w_wait_pub->points.size());

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

  if (!vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_,
                                 LidarMeasures.last_lio_update_time - _first_lidar_time)) {
    pcl_w_wait_pub->clear();
    return;
  }

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  publish_img_rgb(pubImage, vio_manager);

}

void LIVMapper::handleLIO() 
{    
           
  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    ++culvert_empty_lio;
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) 
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    } 
    else 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  publish_odometry(pubOdomAftMapped);
  publishCulvertSource();

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);

  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  if (frame_num == 1 || frame_num % 100 == 0)
    printf("[LIO] frame=%d downsample=%.4f ICP=%.4f map=%.4f total=%.4f avg=%.4f\n",
           frame_num, t_down-t0, t2-t1, t4-t3, t4-t0, aver_time_consu);

}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    std::string raw_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::run(rclcpp::Node::SharedPtr &node) 
{
  rclcpp::WallRate rate(5000);
  RollingCsv eval("Log/processing.csv",
      "wall_s,ros_s,scan_begin_s,scan_end_s,state_s,processing_ms,cycle_ms,frames,lidar_queue,imu_queue,"
      "lidar_rx,imu_rx,image_rx,image_queue,vio_frames,visual_update_frames,visual_update_steps,synced");
  double last_sample = -1.0;
  unsigned long long synced = 0;
  const auto wall = [] {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();};
  while (rclcpp::ok()) 
  {
    const double cycle_begin=wall();
    try {
      rclcpp::spin_some(this->node);
    } catch (const rclcpp::exceptions::RCLError&) {
      // A signal may shut down the context after the while(ok) check.
      if (rclcpp::ok()) throw;
      break;
    }
    if (img_buffer.size()>64 || lid_raw_data_buffer.size()>100 || imu_buffer.size()>10000)
      throw std::runtime_error("Input queues stalled: stopping before memory exhaustion");
    if (!sync_packages(LidarMeasures)) 
    {
      rate.sleep();
      continue;
    }
    const double processing_begin=wall();
    ++synced;
    handleFirstFrame();

    processImu();

    // if (!p_imu->imu_time_init) continue;

    stateEstimationAndMapping();
    const auto sampled_at = wall();
    if (last_sample < 0 || sampled_at - last_sample >= 1.0) {
      last_sample = sampled_at;
      std::lock_guard<std::mutex> guard(mtx_buffer);
      eval.row() << std::setprecision(17) << sampled_at << ',' << this->node->now().seconds() << ',' << LidarMeasures.lidar_frame_beg_time
           << ',' << LidarMeasures.lidar_frame_end_time << ',' << LidarMeasures.last_lio_update_time
           << ',' << (wall()-processing_begin)*1000 << ',' << (wall()-cycle_begin)*1000 << ',' << frame_num
           << ',' << lid_raw_data_buffer.size() << ',' << imu_buffer.size() << ',' << eval_lidar_rx.load() << ',' << eval_imu_rx.load()
           << ',' << eval_image_rx.load() << ',' << img_buffer.size() << ',' << vio_manager->frame_count
           << ',' << vio_manager->visual_update_frames << ',' << vio_manager->visual_update_steps << ',' << synced << std::endl;
    }
  }
  std::ofstream source_counts("Log/culvert_source_counts.json");
  source_counts << std::setprecision(17) << "{\"attempts_after_odom\":" << culvert_attempts
                << ",\"published_pairs\":" << culvert_pairs << ",\"empty_before_odom\":" << culvert_empty_lio
                << ",\"any_unconsumed\":" << culvert_partial << ",\"any_dt_miss\":" << culvert_dt_miss
                << ",\"any_tail_overrun\":" << culvert_overrun << ",\"max_dt_error_s\":" << culvert_max_dt_error_s
                << ",\"max_tail_overrun_s\":" << culvert_max_overrun_s
                << ",\"published_trimmed_frames\":" << culvert_trimmed_frames
                << ",\"published_trimmed_points\":" << culvert_trimmed_points
                << ",\"max_published_trim_fraction\":" << culvert_max_trim_fraction << ",\"first_rejection\":{";
  bool first_reason = true;
  for (const auto &reason : culvert_rejections) {
    if (!first_reason) source_counts << ',';
    source_counts << '"' << reason.first << "\":" << reason.second;
    first_reason = false;
  }
  source_counts << "}}" << std::endl;
  if (!source_counts) RCLCPP_ERROR(node->get_logger(), "Writing Log/culvert_source_counts.json failed");
  std::ofstream counts("Log/input_counters.json");
  counts << "{\"lidar_rx\":" << eval_lidar_rx.load() << ",\"imu_rx\":" << eval_imu_rx.load() << ",\"synced\":" << synced
         << ",\"image_rx\":" << eval_image_rx.load() << ",\"image_remaining\":" << img_buffer.size()
         << ",\"vio_frames\":" << vio_manager->frame_count << ",\"visual_update_frames\":" << vio_manager->visual_update_frames
         << ",\"visual_update_steps\":" << vio_manager->visual_update_steps
         << ",\"lidar_remaining\":" << lid_raw_data_buffer.size() << ",\"imu_remaining\":" << imu_buffer.size() << "}" << std::endl;
  savePCD();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback()
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制 propagate 频率和 IMU 频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && stamp2Sec(prop_imu_buffer.front().header.stamp) < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = stamp2Sec(prop_imu_buffer[i].header.stamp) - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = stamp2Sec(newest_imu.header.stamp) - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom->publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Eigen::Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  ++eval_lidar_rx;
  if (!lidar_en) return;
  mtx_buffer.lock();

  double cur_head_time = stamp2Sec(msg->header.stamp) + lidar_time_offset;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(),"lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  std::sort(ptr->points.begin(), ptr->points.end(), [](const PointType &a, const PointType &b) { return a.curvature < b.curvature; });
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in)
{
  ++eval_lidar_rx;
  if (!lidar_en) return;
  mtx_buffer.lock();
  livox_ros_driver2::msg::CustomMsg::SharedPtr msg(new livox_ros_driver2::msg::CustomMsg(*msg_in));
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - stamp2Sec(msg->header.stamp)) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - stamp2Sec(msg->header.stamp);
    RCLCPP_INFO(this->node->get_logger(), "\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = stamp2Sec(msg->header.stamp);
  RCLCPP_DEBUG(this->node->get_logger(), "Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(), "lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  RCLCPP_DEBUG(this->node->get_logger(), "get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    RCLCPP_ERROR(this->node->get_logger(), "Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  std::sort(ptr->points.begin(), ptr->points.end(), [](const PointType &a, const PointType &b) { return a.curvature < b.curvature; });
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
  ++eval_imu_rx;
  if (!imu_en) return;

  if (last_timestamp_lidar < 0.0) return;
  RCLCPP_DEBUG(this->node->get_logger(), "get imu at time: %.6f", stamp2Sec(msg_in->header.stamp));
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  msg->header.stamp = sec2Stamp(stamp2Sec(msg->header.stamp) - imu_time_offset);
  double timestamp = stamp2Sec(msg->header.stamp);

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    RCLCPP_WARN(this->node->get_logger(), "IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = sec2Stamp(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    RCLCPP_ERROR(this->node->get_logger(), "imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  {
    RCLCPP_WARN(this->node->get_logger(), "imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  RCLCPP_DEBUG(this->node->get_logger(), "IMU stamp %.6f, queue %zu", timestamp, imu_buffer.size());
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
  // A queued BGR image can outlive its ROS message. toCvShare's Mat header
  // alone does not retain the ROS owner when the input already is bgr8.
  return cv_bridge::toCvCopy(img_msg, "bgr8")->image;
}

// static int i = 0;
void LIVMapper::img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in)
{
  ++eval_image_rx;
  if (!img_en) return;
  const auto& msg = msg_in;
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   RCLCPP_WARN(this->node->get_logger(), "img jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }
  // double msg_header_time =  stamp2Sec(msg->header.stamp);
  double msg_header_time = stamp2Sec(msg->header.stamp) + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  RCLCPP_DEBUG(this->node->get_logger(), "Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    RCLCPP_ERROR(this->node->get_logger(), "image loop back. \n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    RCLCPP_WARN(this->node->get_logger(), "Image need Jumps: %.6f", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (!rclcpp::ok()) return false;
  if (img_en && (img_buffer.empty() || img_time_buffer.empty())) return false;
  // The LIO half of a LIVO pair already synchronized the image time against both streams.
  // Completing that pair reads only the retained image, including at the end of a bag.
  const bool pending_vio = slam_mode_ == LIVO && meas.lio_vio_flg == LIO;
  if (!pending_vio &&
      (lid_raw_data_buffer.empty() || lid_header_time_buffer.empty() ||
       ((imu_en || slam_mode_ == LIVO) && imu_buffer.empty()))) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = stamp2Sec(imu_buffer.back()->header.stamp);

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        RCLCPP_ERROR(this->node->get_logger(), "[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        // RCLCPP_ERROR(this->node->get_logger(), "lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty())
      {
        if (stamp2Sec(imu_buffer.front()->header.stamp) > m.lio_time) break;

        if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // stamp2Sec(imu_buffer.front()->header.stamp));
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  RCLCPP_ERROR(this->node->get_logger(), "out sync");
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = this->node->get_clock()->now();
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

// Provide output format for LiDAR-visual BA
void LIVMapper::publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  static int pub_num = 1;
  pub_num++;

  if (LidarMeasures.lio_vio_flg == VIO)
  {
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num >= pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255; else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO)
  {
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  if (slam_mode_ == ONLY_LIO || slam_mode_ == ONLY_LO)
  { 
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  }
  laserCloudmsg.header.stamp = (slam_mode_ == ONLY_LIO ? rclcpp::Time(static_cast<int64_t>(std::llround(LidarMeasures.last_lio_update_time * 1e9)), RCL_ROS_TIME) : this->node->get_clock()->now()); //.fromSec(last_timestamp_lidar);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes->publish(laserCloudmsg);

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    update_time = LidarMeasures.measures.back().lio_time;
  }
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en)
  {
    static int scan_wait_num = 0;

    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        if (slam_mode_ == LIVO)
        {
          *pcl_wait_save += *laserCloudWorldRGB;
        }
        else
        {
          *pcl_wait_save_intensity += *pcl_w_wait_pub;
        }
        if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) scan_wait_num++;
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          *pcl_wait_save_intensity += *laserCloudBody;
          scan_wait_num++;
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        pcd_save_interval = 1;
        
        break;

      default:
        pcd_save_interval = 1;
        scan_wait_num++;
        break;
    }
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") + ss_time.str() + string(".pcd"));

      pcl::PCDWriter pcd_writer;

      cout << "current scan saved to " << all_points_dir << endl;
      if (pcl_wait_save->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
        PointCloudXYZRGB().swap(*pcl_wait_save);
      }
      if(pcl_wait_save_intensity->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      scan_wait_num = 0;
    }
    
    if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos <<  LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
    }
  }
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
  {
    static int img_wait_num = 0;
    img_wait_num++;

    if (img_save_interval > 0 && img_wait_num >= img_save_interval)
    {
      imwrite(string(string(ROOT_DIR) + "Log/image/") + ss_time.str() + string(".png"), vio_manager->img_rgb);
      
      Eigen::Quaterniond q(_state.rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
    }
  }

  if(laserCloudWorldRGB->size() > 0)  PointCloudXYZI().swap(*pcl_wait_pub); 
  if(LidarMeasures.lio_vio_flg == VIO)  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = (slam_mode_ == ONLY_LIO ? rclcpp::Time(static_cast<int64_t>(std::llround(LidarMeasures.last_lio_update_time * 1e9)), RCL_ROS_TIME) : this->node->get_clock()->now());
    laserCloudmsg.header.frame_id = "camera_init";
    pubSubVisualMap->publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = this->node->get_clock()->now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publishCulvertSource()
{
  ++culvert_attempts;
  auto reject = [this](const char *reason) { ++culvert_rejections[reason]; };
  culvert_partial += p_imu->deskew_points_unconsumed;
  culvert_dt_miss += p_imu->deskew_dt_error_s > 1e-6;
  culvert_overrun += p_imu->deskew_window_overrun_s > 1e-6;
  culvert_max_dt_error_s = std::max(culvert_max_dt_error_s, p_imu->deskew_dt_error_s);
  culvert_max_overrun_s = std::max(culvert_max_overrun_s, p_imu->deskew_window_overrun_s);
  if (LidarMeasures.lio_vio_flg != LIO || LidarMeasures.measures.empty()) return reject("not_lio_cycle");
  if (p_imu->deskew_rejection) return reject(p_imu->deskew_rejection);
  if (voxelmap_manager->ptpl_list_.empty()) return reject("no_point_plane_match");
  const double t = p_imu->deskewed_lio_time;
  if (!std::isfinite(t) || t <= 0 || t != LidarMeasures.measures.back().lio_time ||
      t != LidarMeasures.last_lio_update_time) return reject("cycle_time_mismatch");
  if (!_state.pos_end.allFinite() || !_state.rot_end.allFinite()) return reject("nonfinite_pose");
  Eigen::Quaterniond q(_state.rot_end);
  if (!std::isfinite(q.norm()) || q.norm() < 1e-9) return reject("invalid_quaternion");
  q.normalize();

  pcl::PointCloud<pcl::PointXYZI> scan;
  scan.reserve(feats_undistort->size());
  double previous_ms = 0;
  for (const auto &point : *feats_undistort)
  {
    if (!std::isfinite(point.curvature)) return reject("nonfinite_point_time");
    if (point.curvature < previous_ms) return reject("point_time_unordered");
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.intensity)) return reject("nonfinite_xyzi");
    previous_ms = point.curvature;
    // Keep v2's per-point window; omit only certified late points from this copy.
    if (point.curvature * 0.001 > p_imu->deskew_duration_s + 1e-6) continue;
    pcl::PointXYZI p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    p.intensity = point.intensity;
    scan.push_back(p);
  }
  const int64_t stamp_ns = static_cast<int64_t>(std::llround(t * 1e9));
  const size_t total = feats_undistort->size(), removed = total - scan.size();
  if (removed) {
    const double fraction = static_cast<double>(removed) / total;
    const char *failure = nullptr;
    if (slam_mode_ != LIVO || !p_imu->deskew_same_origin) failure = "uncertain_window_origin";
    else if (p_imu->deskew_window_overrun_s > 0.0005) failure = "excessive_tail_time";
    else if (fraction > 0.01 || scan.empty()) failure = "excessive_tail_fraction";
    static RollingCsv tail("Log/culvert_source_tail.csv",
        "stamp_ns,input_points,outside_points,inside_points,outside_fraction,tail_overrun_s,decision");
    tail.row() << std::setprecision(17) << stamp_ns << ',' << total << ',' << removed << ','
               << scan.size() << ',' << fraction << ',' << p_imu->deskew_window_overrun_s
               << ',' << (failure ? failure : "publish_trimmed") << '\n';
    if (failure) return reject(failure);
    ++culvert_trimmed_frames;
    culvert_trimmed_points += removed;
    culvert_max_trim_fraction = std::max(culvert_max_trim_fraction, fraction);
  }
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(scan, cloud);
  cloud.header.frame_id = "livo_lidar";
  cloud.header.stamp = rclcpp::Time(stamp_ns, RCL_ROS_TIME);

  nav_msgs::msg::Odometry state;
  state.header.stamp = cloud.header.stamp;
  state.header.frame_id = "camera_init";
  state.child_frame_id = "imu";
  auto &pose = state.pose.pose;
  pose.position.x = _state.pos_end.x();
  pose.position.y = _state.pos_end.y();
  pose.position.z = _state.pos_end.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  pubCulvertState->publish(state);
  pubCulvertScan->publish(cloud);
  ++culvert_pairs;
}

void LIVMapper::publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = (slam_mode_ == ONLY_LIO ? rclcpp::Time(static_cast<int64_t>(std::llround(LidarMeasures.last_lio_update_time * 1e9)), RCL_ROS_TIME) : this->node->get_clock()->now()); //.ros::Time()fromSec(last_timestamp_lidar);
  set_posestamp(odomAftMapped.pose.pose);

  static std::shared_ptr<tf2_ros::TransformBroadcaster> br;
  br = std::make_shared<tf2_ros::TransformBroadcaster>(this->node);
  tf2::Transform transform;
  tf2::Quaternion q;
  transform.setOrigin(tf2::Vector3(_state.pos_end(0), _state.pos_end(1), _state.pos_end(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);
  br->sendTransform(geometry_msgs::msg::TransformStamped(createTransformStamped(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped")));
  pubOdomAftMapped->publish(odomAftMapped);
}

void LIVMapper::publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = (slam_mode_ == ONLY_LIO ? rclcpp::Time(static_cast<int64_t>(std::llround(LidarMeasures.last_lio_update_time * 1e9)), RCL_ROS_TIME) : this->node->get_clock()->now());
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void LIVMapper::publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = (slam_mode_ == ONLY_LIO ? rclcpp::Time(static_cast<int64_t>(std::llround(LidarMeasures.last_lio_update_time * 1e9)), RCL_ROS_TIME) : this->node->get_clock()->now());
  msg_body_pose.header.frame_id = "camera_init";
  path.header = msg_body_pose.header;
  if (path.poses.size() >= 10000) path.poses.erase(path.poses.begin(), path.poses.begin()+1000);
  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
}
