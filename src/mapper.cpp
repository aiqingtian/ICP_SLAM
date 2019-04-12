#include <icp_slam/mapper.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <cstdlib>


namespace icp_slam
{
Mapper::Mapper()
	:is_initialized_(false), numlaserScan(1)
{
}

void Mapper::initMap(int width, int height, float resolution,
               double origin_x_meters, double origin_y_meters,
               uint8_t *pointer, unsigned char unknown_cost_value)
{
	resolution_ = resolution;
	height_ = height;
	width_ = width;
	map_.create(height_, width_, CV_8SC1);
	for(int i= 0; i<=height_; i++)
	{
		for(int j=0; j<= width_; j++)
		{
			map_.at<int8_t>(i, j) = -1;
		}
	}
    relative_map_.create(height_, width_, CV_8SC1);
	for(int i= 0; i<=height_; i++)
	{
		for(int j=0; j<= width_; j++)
		{
			relative_map_.at<int8_t>(i, j) = 0;
		}
	}
	origin_x_ = origin_x_meters;
	origin_y_ = origin_y_meters;
	is_initialized_ = true;
}

cv::Mat Mapper::getMapCopy()
{
	cv::Mat map__; 
	map_.copyTo(map__);
	return map__;
}

int Mapper::updateMap(const sensor_msgs::LaserScanConstPtr &laser_scan,
                const tf::StampedTransform &pose)
{	
	boost::unique_lock<boost::recursive_mutex> scoped_lock(mutex_);
	robot_pose_.x = pose.getOrigin().getX();
	robot_pose_.y = pose.getOrigin().getY();
	robot_pose_.a = tf::getYaw(pose.getRotation());
	updateLaserScan(laser_scan, robot_pose_);
	return 1;
}

int Mapper::updateLaserScan(const sensor_msgs::LaserScanConstPtr &laser_scan, robot_pose_t robot_pose)
{
	sensor_msgs::LaserScan laser_scan_;
    laser_scan_ = *laser_scan;
	int ranges_size = laser_scan_.ranges.size();
	double angle_min = laser_scan_.angle_min;
	double angle_max = laser_scan_.angle_max;
	for(int i = 0; i< ranges_size; i++)
	{
		double ranges_ = laser_scan_.ranges[i];
		double theta = angle_min + robot_pose.a;
		angle_min += laser_scan_.angle_increment; 
		if(ranges_ < laser_scan_.range_min | ranges_ > laser_scan_.range_max)
		{
		}
		if(std::isnan(ranges_))
		{
			continue;
		}
		double x_w = ranges_ * std::cos(theta) + robot_pose.x;
		double y_w = ranges_ * std::sin(theta) + robot_pose.y;
		// Marks
		int grid_x;
		int grid_y;
		int grid_x0;
		int grid_y0;
		convertToGridCoords(x_w, y_w, grid_x, grid_y);
		convertToGridCoords(robot_pose.x, robot_pose.y, grid_x0, grid_y0);
		if (grid_x > height_ | grid_x0> height_ | grid_y> width_ | grid_y0 > width_)
		{
			continue;
		}
		cv::LineIterator it(map_, cv::Point(grid_x0, grid_y0), cv::Point(grid_x, grid_y));
		for(int j =0; j< it.count; j++, ++it)
		{
			cv::Point point = it.pos();
			int pointx = point.x;
			int pointy = point.y;
			if(pointx >= height_ | pointy >= width_)
			{
				continue;
			}
			if(j < it.count -1)
			{
				if(relative_map_.at<int8_t>(pointx, pointy) >124)
				{
					relative_map_.at<int8_t>(pointx, pointy) =124;
				}else
				{
					relative_map_.at<int8_t>(pointx, pointy) += 1;

				}
					
					map_.at<int8_t>(pointx, pointy) =0;

			}else if(j >= it.count -1)
			{
				if(relative_map_.at<int8_t>(pointx, pointy) <-124)
				{
					relative_map_.at<int8_t>(pointx, pointy) =-124;
				}else
				{
					relative_map_.at<int8_t>(pointx, pointy) -= 1;
				}
				map_.at<int8_t>(pointx, pointy) =100;
				
			}

		}
	}
	for(int i=0; i<height_; i++)
	{
		for(int j=0; j<width_; j++)
		{
			if(relative_map_.at<int8_t>(i,j) >= 50)
			{
				map_.at<int8_t>(i, j) = 0;
			}
			else if (relative_map_.at<int8_t>(i, j) <=-50)
			{
				map_.at<int8_t>(i, j) =100;
			}
		}
	}
	numlaserScan +=1;
	return 1;
}

// Convert world coordinate to grid coordinate
int Mapper::convertToGridCoords(double x, double y, int &grid_x, int &grid_y)
{
	double dist_x = fabs(x + origin_x_);
	double dist_y = fabs(y + origin_y_);
	grid_x = (int)(dist_x/resolution_);
	grid_y = (int)(dist_y/resolution_);
	return 1;
}
} // namespace icp_slam