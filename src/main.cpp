#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
#include "RRT.h"
#include "OccMapTransform.h"


//----------------------Global variables-------------------//
// Subscriber
ros::Subscriber map_sub;
ros::Subscriber startPoint_sub;
ros::Subscriber targetPoint_sub;
// Publisher
ros::Publisher mask_pub;
ros::Publisher path_pub;

//Object
nav_msgs::OccupancyGrid OccGridMask;
nav_msgs::Path path;
pathplanning::RRTConfig config;
pathplanning::RRT rrt;
OccupancyGridParam OccGridParam;
cv::Point startPoint,targetPoint;

// Parameter
double inflateRaadius;
bool map_flag;
bool startpoint_flag;
bool targetpoint_flag;
bool start_flag;
int rate;


//-----------------Callback Function---------------------//
void MapCallback(const nav_msgs::OccupancyGrid& msg)
{
    ROS_INFO("mapcallback");
    //get parameter
    OccGridParam.GetOccupancyGridParam(msg);

    //get map
    int height = OccGridParam.height;
    int width = OccGridParam.width;
    int OccProb;
    cv::Mat Map(height,width,CV_8UC1);
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++){
            OccProb = msg.data[i*width+j];
            OccProb = (OccProb<0) ? 100:OccProb;  
            //set unknowed to occupied; unknow:-1,free:0,occupied:100
            Map.at<uchar>(height-i-1,j) = 255 -round(OccProb*255/100);
            //the occupied position of map is set to 0,elif is set to 255 
        }
    }

    //initial RRT
    cv::Mat Mask;
    config.InflateRadius = round(inflateRaadius / OccGridParam.resolution);
    rrt.InitRRT(Map,Mask,config);
    ROS_INFO("initial RRT");

    //Publish Mask
    OccGridMask.header.stamp = ros::Time::now();
    OccGridMask.header.frame_id = "map";
    OccGridMask.info = msg.info;
    OccGridMask.data.clear();
    for(int i=0;i<height;i++){
        for(int j=0;j<width;j++){
            OccProb = Mask.at<uchar>(height-i-1,j)*255;
            OccGridMask.data.push_back(OccProb);
        }
    }

    map_flag = true;
    startpoint_flag = false;
    targetpoint_flag = false;
    ROS_INFO("Mapcallback end");
}

void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    ROS_INFO("StartPointCallback");
    cv::Point2d src_point(msg.pose.pose.position.x,msg.pose.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point,startPoint);

    //set flag
    startpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag){
        start_flag = true;
    }
    ROS_INFO("startPoint: %f %f %d %d", msg.pose.pose.position.x, msg.pose.pose.position.y,
             startPoint.x, startPoint.y);
}

void TargetPointCallback(const geometry_msgs::PoseStamped& msg)
{
    ROS_INFO("TargetPointCallback");
    cv::Point2d src_point(msg.pose.position.x,msg.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point,targetPoint);

    //set flag
    targetpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }
    ROS_INFO("targetPoint: %f %f %d %d", msg.pose.position.x, msg.pose.position.y,
            targetPoint.x, targetPoint.y);


}

int main(int argc,char * argv[])
{
    //Initiate node
    ros::init(argc,argv,"rrt");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start rrt node \n");

    // Initiate variables
    map_flag = false;
    startpoint_flag = false;
    targetpoint_flag = false;
    start_flag = false;

    //parameter
    nh_priv.param<bool>("Euclidean",config.Euclidean,true);
    nh_priv.param<int>("OccupyThresh",config.OccupyThresh,-1);
    nh_priv.param<double>("InflateRadius",inflateRaadius,-1);
    nh_priv.param<int>("rate",rate,10);
    nh_priv.param<double>("goal_sample_rate",config.goal_sample_rate,0.05);
    nh_priv.param<double>("extend_dis",config.extend_dis,10);
    nh_priv.param<int>("max_iteration", config.max_iteration,3000);


    //Subscribe topics
    map_sub = nh.subscribe("map",10,MapCallback);
    startPoint_sub = nh.subscribe("initialpose",10,StartPointCallback);
    targetPoint_sub = nh.subscribe("move_base_simple/goal",10,TargetPointCallback);

    //Advertise topics
    mask_pub = nh.advertise<nav_msgs::OccupancyGrid>("mask",1);
    path_pub = nh.advertise<nav_msgs::Path>("nav_path",10);

    //loop and wait for callback
    ROS_INFO("wait for callback");
    ros::Rate loop_rate(rate);
    while(ros::ok()){
        if(start_flag){
            ROS_INFO("loop start");
            double start_time = ros::Time::now().toSec();

            ROS_INFO("start planning path");
            std::deque<cv::Point> PathList;
            rrt.PathPlanning(startPoint,targetPoint,PathList);
            if(!PathList.empty())
            {
                path.header.stamp = ros::Time::now();
                path.header.frame_id = "map";
                path.poses.clear();
                for(int i = 0;i<PathList.size();i++){
                    cv::Point2d dst_point;
                    OccGridParam.Image2MapTransform(PathList[i],dst_point);

                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "map";
                    pose_stamped.pose.position.x = dst_point.x;
                    pose_stamped.pose.position.y = dst_point.y;
                    pose_stamped.pose.position.x = 0; 
                    path.poses.push_back(pose_stamped);
                }
                path_pub.publish(path);
                double end_time = ros::Time::now().toSec();
                ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
            }
            else{
                ROS_ERROR("Can not find a valid path.");
            }

            start_flag = false;
        }
        if(map_flag)
        {
            mask_pub.publish(OccGridMask);
            // ROS_INFO("mask published");
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}