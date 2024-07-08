//created by llh on 24-7-4


//问题：创建出的地图的原点是怎么确定的？
#include "OccMapTransform.h"

void OccupancyGridParam::GetOccupancyGridParam(const nav_msgs::OccupancyGrid& OccGrid)
{
    // get parameter
    resolution = OccGrid.info.resolution;
    height = OccGrid.info.height;
    width = OccGrid.info.width;
    x = OccGrid.info.origin.position.x;
    y = OccGrid.info.origin.position.y;

    double roll,pith,yaw;
    geometry_msgs::Quaternion q = OccGrid.info.origin.orientation;
    tf::Quaternion quat(q.x,q.y,q.z,q.w);
    tf::Matrix3x3(quat).getRPY(roll,pith,yaw);
    theta = yaw;

    //Calculate R,t
    R = cv::Mat::zeros(2,2,CV_64FC1);
    R.at<double>(0,0) = resolution * cos(theta);
    R.at<double>(0,1) = resolution * sin(-theta);
    R.at<double>(1,0) = resolution * sin(theta);
    R.at<double>(1,1) = resolution * cos(theta);
    t = cv::Mat(cv::Vec2d(x,y),CV_64FC1);
}

void OccupancyGridParam::Image2MapTransform(cv::Point& src_point,cv::Point2d& dst_point)
{
    //图像坐标系的原点在左上角
    cv::Mat P_src(cv::Vec2d(src_point.x,height-1-src_point.y),CV_64FC1);
    cv::Mat P_dst = R*P_src +t;

    dst_point.x = P_dst.at<double>(0,0);
    dst_point.y = P_dst.at<double>(1,0);
}

void OccupancyGridParam::Map2ImageTransform(cv::Point2d& src_point,cv::Point& dst_point)
{
    cv::Mat P_src(cv::Vec2d(src_point.x, src_point.y), CV_64FC1);
    cv::Mat P_dst = R.inv()*(P_src - t);
    dst_point.x = round(P_dst.at<double>(0,0));
    dst_point.y = height - 1- round(P_dst.at<double>(1,0));    
}