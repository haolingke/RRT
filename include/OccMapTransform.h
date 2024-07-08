// created by lilinghao on 24-7-4

#ifndef OccMapTransform_H
#define OccMapTransform_H

#include <iostream>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>

class OccupancyGridParam{
public:
    //variable
    int width;
    int height;
    double resolution;
    //origin pose;
    double x;
    double y;
    double theta;

    //function
    void GetOccupancyGridParam(const nav_msgs::OccupancyGrid& msg);
    void Image2MapTransform(cv::Point& PathPoint,cv::Point2d& dst_point);
    void Map2ImageTransform(cv::Point2d& dst_point,cv::Point& PathPoint);

private:
    cv::Mat R;
    cv::Mat t;
};



#endif 