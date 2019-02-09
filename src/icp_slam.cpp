//
// Created by rakesh on 13/08/18.
//
#include <cmath>
#include <map>
#include <numeric>
#include <chrono>

#include <boost/atomic/atomic.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <icp_slam/icp_slam.h>
#include <icp_slam/utils.h>
#include <icp_slam/config.h>

#define TIME_DIFF(tic, toc) ((std::chrono::duration<double, std::milli>((toc) - (tic))).count())

namespace icp_slam
{

ICPSlam::ICPSlam(tfScalar max_keyframes_distance, tfScalar max_keyframes_angle, double max_keyframes_time)
  : max_keyframes_distance_(max_keyframes_distance),
    max_keyframes_angle_(max_keyframes_angle),
    max_keyframes_time_(max_keyframes_time),
    last_kf_laser_scan_(new sensor_msgs::LaserScan()),
    is_tracker_running_(false)
{
  last_kf_tf_odom_laser_.stamp_ = ros::Time(0);
}

bool ICPSlam::track(const sensor_msgs::LaserScanConstPtr &laser_scan,
                    const tf::StampedTransform &current_frame_tf_odom_laser,
                    tf::StampedTransform &tf_map_laser)
{
  if (is_tracker_running_)
  {
    ROS_WARN_THROTTLE(1.0, "Couldn't track frame, tracker already running");
    return false;
  }

  // TODO: find the pose of laser in map frame
  // if a new keyframe is created, run ICP
  // if not a keyframe, obtain the laser pose in map frame based on odometry update
  is_tracker_running_ = true;

  // initialize last_kf_tf_odom_laser
  if (last_kf_tf_odom_laser_.frame_id_.compare("") == 0){
    ROS_WARN("Initialize\r");
    last_kf_tf_map_laser_ = current_frame_tf_odom_laser;
    last_kf_tf_map_laser_.frame_id_ = "map";
    last_kf_tf_map_laser_.child_frame_id_ = "odom";
    last_kf_tf_odom_laser_ = current_frame_tf_odom_laser;
    *last_kf_laser_scan_ = *laser_scan;
    tf_map_laser = current_frame_tf_odom_laser;
  }

  tf::Transform T_2_1 = last_kf_tf_odom_laser_.inverse() * current_frame_tf_odom_laser;
  // tf::Transform T_2_1 = current_frame_tf_odom_laser * last_kf_tf_odom_laser_.inverse();

  if (isCreateKeyframe(current_frame_tf_odom_laser, last_kf_tf_odom_laser_) == false){
    tf_map_laser = tf::StampedTransform(last_kf_tf_map_laser_ * T_2_1, ros::Time::now(), last_kf_tf_map_laser_.frame_id_, last_kf_tf_map_laser_.child_frame_id_);
    // tf_map_laser = current_frame_tf_odom_laser;
  }else{
    // ROS_WARN("is_keyframe\r");
    // ROS_INFO("T_2_1: (%f, %f), %f",
    //       T_2_1.getOrigin().getX(), 
    //       T_2_1.getOrigin().getY(), 
    //       // tf::getYaw(tf_odom_laser.getRotation()) * 180 / M_PI);
    //       tf::getYaw(T_2_1.getRotation()));

    // tf::Transform refined_T_2_1 = icpRegistration(last_kf_laser_scan_, laser_scan, T_2_1);
    tf::Transform refined_T_2_1 = icpRegistration(laser_scan, last_kf_laser_scan_, T_2_1);
    // if use map laser works not good
    tf_map_laser = tf::StampedTransform(last_kf_tf_map_laser_ * refined_T_2_1, ros::Time::now(), last_kf_tf_map_laser_.frame_id_, last_kf_tf_map_laser_.child_frame_id_);
    
    // ROS_WARN("icpRegistrationfinished\r");
    last_kf_tf_map_laser_ = tf_map_laser;
    last_kf_tf_odom_laser_ = current_frame_tf_odom_laser;
    *last_kf_laser_scan_ = *laser_scan;

  }
  is_tracker_running_ = false;
  return true;
}

bool ICPSlam::isCreateKeyframe(const tf::StampedTransform &current_frame_tf, const tf::StampedTransform &last_kf_tf) const
{
  assert(current_frame_tf.frame_id_ == last_kf_tf.frame_id_);
  assert(current_frame_tf.child_frame_id_ == last_kf_tf.child_frame_id_);
  // TODO: check whether you want to create keyframe (based on max_keyframes_distance_, max_keyframes_angle_, max_keyframes_time_)
  double square_distance_ = 0.0;
  square_distance_ = pow(current_frame_tf.getOrigin().getX() - last_kf_tf.getOrigin().getX(), 2) 
                   + pow(current_frame_tf.getOrigin().getY() - last_kf_tf.getOrigin().getY(), 2);
  if (square_distance_ > pow(max_keyframes_distance_, 2)){
    // ROS_WARN("is Keyframe for distance");
    return true;
  }
  double change_rotation_ = 0.0;
  change_rotation_ = tf::getYaw(current_frame_tf.getRotation()) - tf::getYaw(last_kf_tf.getRotation());
  if (change_rotation_ > max_keyframes_angle_ || change_rotation_ < -max_keyframes_angle_){
    // ROS_WARN("is Keyframe for rotation");
    return true;
  }
  if((current_frame_tf.stamp_ - last_kf_tf.stamp_).toSec() > max_keyframes_time_){
    // ROS_WARN("is Keyframe for time");
    return true;
  }
  return false;
}

tf::Transform ICPSlam::icpRegistration(const sensor_msgs::LaserScanConstPtr &laser_scan1,
                                      const sensor_msgs::LaserScanConstPtr &laser_scan2,
                                      const tf::Transform &T_2_1)
{
  // ROS_WARN("icpRegistration\r");
  cv::Mat laser_scan_mat1 = utils::laserScanToPointMat(laser_scan1);
  cv::Mat laser_scan_mat2 = utils::laserScanToPointMat(laser_scan2);
  if (laser_scan_mat1.rows == 0){
    return T_2_1;
  }
  
  // ROS_WARN("current:(%f,%f),(%f,%f),(%f,%f)]\r", 
  //         laser_scan_mat2.at<float>(0,0), laser_scan_mat2.at<float>(0,1),
  //         laser_scan_mat2.at<float>(63,0), laser_scan_mat2.at<float>(63,1),
  //         laser_scan_mat2.at<float>(126,0), laser_scan_mat2.at<float>(126,1));
  
  std::vector<int> closest_idx;
  std::vector<float> closest_distances;
  
  // Initialize map transform with odom transform
  tf::Transform current_T_2_1 = T_2_1;
  cv::Mat transformed_laser_scan_mat1;
  // TODO: icpIteration
  double mean_square_error;
  double last_mean_square_error = 10000;
  std::vector<cv::Mat> laser_scan_mat1_matched_vec;
  std::vector<cv::Mat> laser_scan_mat2_matched_vec;
  std::vector<tf::Transform> current_T_2_1_vec;
  for (int loop_count = 0; loop_count < 30; loop_count++)
  {
    transformed_laser_scan_mat1 = utils::transformPointMat(current_T_2_1, laser_scan_mat1);
    ICPSlam::closestPoints(transformed_laser_scan_mat1, laser_scan_mat2, closest_idx, closest_distances);
    float mean;
    float std_dev;
    std::vector<int> out_idx;
    utils::meanAndStdDev(closest_distances, mean, std_dev);

    for (int i = 0; i < closest_distances.size(); i++)
    {
      if (closest_distances[i] > (mean + 2 * std_dev))
      {
        out_idx.push_back(i);
        continue;
      }
        mean_square_error += pow(closest_distances[i], 2);
    }
    mean_square_error /= (laser_scan_mat1.rows - out_idx.size());
    

    // Resort laser_scan_mat2 to match laser_scan_mat1 and do outlier rejection
    cv::Mat laser_scan_mat1_matched((laser_scan_mat1.rows - out_idx.size()), 2, CV_32F, cv::Scalar(1.0f));
    cv::Mat laser_scan_mat2_matched((laser_scan_mat1.rows - out_idx.size()), 2, CV_32F, cv::Scalar(1.0f));
    ROS_WARN("iteration:%d, mean_square_error:%f, last_error:%f, out_idx_length:%lu, laser_scan_mat1_matched:%d", 
            loop_count, mean_square_error, last_mean_square_error, out_idx.size(), laser_scan_mat1_matched.rows);
    int row_count_matched = 0;
    for (int row_count = 0; row_count < closest_idx.size(); row_count++)
    {
      bool is_exist = false;
      for (int vec_count = 0; vec_count < out_idx.size(); vec_count++){
        if (out_idx[vec_count] == row_count){
          is_exist = true;
          break;
        }
      }
      if (is_exist == false){
        laser_scan_mat1.row(row_count).copyTo(laser_scan_mat1_matched.row(row_count_matched));
        laser_scan_mat2.row(closest_idx[row_count]).copyTo(laser_scan_mat2_matched.row(row_count_matched));
        row_count_matched++;
      }
    }

    laser_scan_mat1_matched_vec.push_back(laser_scan_mat1_matched);
    laser_scan_mat2_matched_vec.push_back(laser_scan_mat2_matched);
    current_T_2_1_vec.push_back(current_T_2_1);

    // Decide whether we can stop iteration by calculating Error with last T_2_1
    if (fabs(mean_square_error - last_mean_square_error) < 0.0001 || mean_square_error < 0.01){
      break;
    }
    last_mean_square_error = mean_square_error;
    // if (last_mean_square_error < 0.001 && mean_square_error < 0.001){
    //   break;
    // }

    // Do one ICP step
    current_T_2_1 = icpIteration(laser_scan_mat1_matched, laser_scan_mat2_matched);
  
  }
  if (current_T_2_1_vec.size() > 5){
    for (int i = 0; i < current_T_2_1_vec.size(); i++){
      ICPSlam::vizClosestPoints(laser_scan_mat1_matched_vec[i], laser_scan_mat2_matched_vec[i], current_T_2_1_vec[i], std::to_string(i));
    }
  }


  // transformed_laser_scan_mat1 = utils::transformPointMat(current_T_2_1, laser_scan_mat1);
  // ICPSlam::closestPoints(transformed_laser_scan_mat1, laser_scan_mat2, closest_idx, closest_distances);
  // cv::Mat laser_scan_mat2_matched(laser_scan_mat1.rows, 2, CV_32F, cv::Scalar(1.0f));
  // for (int row_count = 0; row_count < laser_scan_mat2_matched.rows; row_count++)
  //   laser_scan_mat2.row(closest_idx[row_count]).copyTo(laser_scan_mat2_matched.row(row_count));
  // ICPSlam::vizClosestPoints(laser_scan_mat1, laser_scan_mat2_matched, current_T_2_1, "1");
  // ICPSlam::vizClosestPoints(laser_scan_mat1, laser_scan_mat2_matched, last_T_2_1, "0");
  return current_T_2_1;
}

tf::Transform ICPSlam::icpIteration(cv::Mat &point_mat1, cv::Mat &point_mat2)
{
  std::vector<float> miu_x(2);
  std::vector<float> miu_p(2);
  cv::Mat cp_point_mat1;
  cv::Mat cp_point_mat2;
  point_mat1.copyTo(cp_point_mat1);
  point_mat2.copyTo(cp_point_mat2);
  miu_x[0] = cv::sum(cp_point_mat1.col(0))[0] / cp_point_mat1.rows;
  miu_x[1] = cv::sum(cp_point_mat1.col(1))[0] / cp_point_mat1.rows;
  miu_p[0] = cv::sum(cp_point_mat2.col(0))[0] / cp_point_mat2.rows;
  miu_p[1] = cv::sum(cp_point_mat2.col(1))[0] / cp_point_mat2.rows;
  cv::subtract(cp_point_mat1.col(0), miu_x[0], cp_point_mat1.col(0));
  cv::subtract(cp_point_mat1.col(1), miu_x[1], cp_point_mat1.col(1));
  cv::subtract(cp_point_mat2.col(0), miu_p[0], cp_point_mat2.col(0));
  cv::subtract(cp_point_mat2.col(1), miu_p[0], cp_point_mat2.col(1));
  
  // point_mat_1 is N*2, need to be 2*N
  cp_point_mat1 = cp_point_mat1.t();
  cv::Mat W = cp_point_mat1 * cp_point_mat2;
  cv::SVD svd(W);
  cv::Mat RR = svd.u * svd.vt;
  cv::Mat miu_x_mat(2, 1, CV_32F);
  memcpy(miu_x_mat.data, miu_x.data(), miu_x.size()*sizeof(float));
  cv::Mat miu_p_mat(2, 1, CV_32F);
  memcpy(miu_p_mat.data, miu_p.data(), miu_p.size()*sizeof(float));
  cv::Mat tt = miu_x_mat - RR * miu_p_mat;
  tf::Transform tf_2_1;
  // tf::Matrix3x3 R_3d(RR.at<float>(0, 0), RR.at<float>(0, 1), 0,
  //                    RR.at<float>(1, 0), RR.at<float>(1, 1), 0,
  //                    0, 0, 1);
  
  double cos_theta = RR.at<float>(0, 0);
  double sin_theta = RR.at<float>(0, 1);
  double theta = atan2(sin_theta, cos_theta);

  tf_2_1.setOrigin(tf::Vector3(tt.at<float>(0, 0), tt.at<float>(0, 1), 0.0));
  // tf_2_1.setBasis(R_3d);
  tf_2_1.setRotation(tf::createQuaternionFromYaw(theta));

  return tf_2_1;
}

void ICPSlam::closestPoints(cv::Mat &point_mat1,
                            cv::Mat &point_mat2,
                            std::vector<int> &closest_indices,
                            std::vector<float> &closest_distances_2)
{
  // uses FLANN for K-NN e.g. http://www.morethantechnical.com/2010/06/06/iterative-closest-point-icp-with-opencv-w-code/
  closest_indices = std::vector<int>(point_mat1.rows, -1);
  closest_distances_2 = std::vector<float>(point_mat1.rows, -1);


  cv::Mat multi_channeled_mat1;
  cv::Mat multi_channeled_mat2;

  point_mat1.convertTo(multi_channeled_mat1, CV_32FC2);
  point_mat2.convertTo(multi_channeled_mat2, CV_32FC2);

  cv::flann::Index flann_index(multi_channeled_mat2, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees

  cv::Mat mat_indices(point_mat1.rows, 1, CV_32S);
  cv::Mat mat_dists(point_mat1.rows, 1, CV_32F);
  flann_index.knnSearch(multi_channeled_mat1, mat_indices, mat_dists, 1, cv::flann::SearchParams(64) );

  int* indices_ptr = mat_indices.ptr<int>(0);
  //float* dists_ptr = mat_dists.ptr<float>(0);
  for (int i=0;i<mat_indices.rows;++i) {
    closest_indices[i] = indices_ptr[i];
  }

  mat_dists.copyTo(cv::Mat(closest_distances_2));

  // ---------------------------- naive version ---------------------------- //
  // max allowed distance between corresponding points
//  const float max_distance = 0.5;
//
//  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
//  {
//    int closest_point_idx = -1;
//    float closest_distance_2 = std::pow(max_distance, 2.0f);
//
//    for (size_t j = 0, len_j = (size_t)point_mat2.rows; j < len_j; j++)
//    {
//      auto distance2 =
//        std::pow(point_mat2.at<float>(j, 0) - point_mat1.at<float>(i, 0), 2.0f)
//        + std::pow(point_mat2.at<float>(j, 1) - point_mat1.at<float>(i, 1), 2.0f);
//
//      if (distance2 < closest_distance_2)
//      {
//        closest_distance_2 = distance2;
//        closest_point_idx = (int)j;
//      }
//    }
//
//    if (closest_point_idx >= 0)
//    {
//      closest_indices[i] = closest_point_idx;
//      closest_distances_2[i] = closest_distance_2;
//    }
//  }
}

void ICPSlam::vizClosestPoints(cv::Mat &point_mat1,
                               cv::Mat &point_mat2,
                               const tf::Transform &T_2_1,
                               std::string name
                               )
{
  assert(point_mat1.size == point_mat2.size);

  const float resolution = 0.005;

  float *float_array = (float*)(point_mat1.data);
  float size_m = std::accumulate(
    float_array, float_array + point_mat1.total(), std::numeric_limits<float>::min(),
    [](float max, float current)
    {
      return current > max ? current : max;
    }
  );
  // add some slack
  size_m += 0.5;

  int size_pix = (int)(size_m / resolution);

  cv::Mat img(
    size_pix,
    size_pix,
    CV_8UC3,
    cv::Scalar(0, 0, 0)
  );

  auto meters_to_pix = [&size_pix, resolution](float meters) {
    int pix = (int)(meters / resolution + size_pix / 2.0f);
    pix = std::max(0, pix);
    pix = std::min(size_pix - 1, pix);
    return pix;
  };

  cv::Mat transformed_point_mat2 = utils::transformPointMat(T_2_1.inverse(), point_mat2);

  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
  {
    float x1 = point_mat1.at<float>(i, 0);
    float y1 = point_mat1.at<float>(i, 1);
    float x2 = transformed_point_mat2.at<float>(i, 0);
    float y2 = transformed_point_mat2.at<float>(i, 1);

    auto pix_x1 = meters_to_pix(x1);
    auto pix_y1 = meters_to_pix(y1);
    auto pix_x2 = meters_to_pix(x2);
    auto pix_y2 = meters_to_pix(y2);

    cv::Point point1(pix_x1, pix_y1);
    cv::Point point2(pix_x2, pix_y2);

    // cv::circle(img, point1, 5, cv::Scalar(0, 0, 255), -1);
    cv::circle(img, point1, 5, cv::Scalar(0, 0, 255), -1);
    cv::circle(img, point2, 5, cv::Scalar(234, 242, 248), -1);

    cv::line(img, point1, point2, cv::Scalar(0, 255, 0), 2);
  }

  cv::Mat tmp;
  cv::flip(img, tmp, 0);
  cv::imwrite("/home/songyih/icp/icp_laser"+name+".png", img);
}

} // namespace icp_slam

