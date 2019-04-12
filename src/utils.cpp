#include <icp_slam/utils.h>

namespace icp_slam
{
namespace utils
{

cv::Mat laserScanToPointMat(const sensor_msgs::LaserScanConstPtr &scan)
{
  sensor_msgs::LaserScan laser_scan_;
  laser_scan_ = *scan;
  // for stimulator, scan_ranges_len is 128, laser_scan_.angle_min is -3.14, laser_scan_.angle_max is 3.14
  float x;
  float y;
  std::vector<float> x_vec;
  std::vector<float> y_vec;
  int idx = 0;
  for (float i = laser_scan_.angle_min; i < laser_scan_.angle_max; i += laser_scan_.angle_increment){
  	if (idx >= laser_scan_.ranges.size())
  		break;
  	if (laser_scan_.ranges[idx] < laser_scan_.range_min || laser_scan_.ranges[idx] > laser_scan_.range_max){
      laser_scan_.ranges[idx] = laser_scan_.range_max;
    }
  	polarToCartesian(laser_scan_.ranges[idx], i, x, y);
  	x_vec.push_back(x);
  	y_vec.push_back(y);
    idx++;
  }
  
  cv::Mat M_x(x_vec, true);
  cv::Mat M_y(y_vec, true);
  assert(M_x.rows == M_y.rows);

  cv::Mat point_mat_homogeneous(M_x.rows, 2, CV_32F, cv::Scalar(1.0f));
  if (point_mat_homogeneous.rows == 0){
    return point_mat_homogeneous;
  }
  M_x.copyTo(point_mat_homogeneous.col(0));
  M_y.copyTo(point_mat_homogeneous.col(1));
  return point_mat_homogeneous;
}

cv::Mat transformPointMat(tf::Transform transform, cv::Mat &point_mat)
{
  assert(point_mat.data);
  assert(!point_mat.empty());

  cv::Mat point_mat_homogeneous(3, point_mat.rows, CV_32F, cv::Scalar(1.0f));

  cv::Mat(point_mat.t()).copyTo(point_mat_homogeneous.rowRange(0, 2));

  auto T = transformToMatrix(transform);
  cv::Mat transformed_point_mat =  T * point_mat_homogeneous;
  return cv::Mat(transformed_point_mat.t()).colRange(0, 2);
}

} // namespace utils
} // namespace icp_slam