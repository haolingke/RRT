// created by lilinghao on 24-7-4

#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <queue>
#include <random>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <cmath>

namespace pathplanning{

enum NodeType{
    obstacle = 0,
    free,
    inOpenList,
    inCloseList
};

class Node{
public:
    cv::Point point;
    //float G;
    Node* parent;
    Node(cv::Point _point,Node* _parent):point(_point),parent(_parent){}
   
};

class cmp{
public:
    bool operator()(std::pair<int,cv::Point> a,std::pair<int,cv::Point> b){
        return a.first > b.first;  
    }    
};

class RRTConfig{
public:
    bool Euclidean;   
    int OccupyThresh;
    int InflateRadius;
    double goal_sample_rate;  //随机采样到目标点的概率
    double extend_dis;  //树枝长度
    double rand_area;
    int max_iteration; //最大迭代次数

    RRTConfig(bool _Euclidean = true, int _OccupyThresh = -1, int _InflateRadius = -1):
        Euclidean(_Euclidean), OccupyThresh(_OccupyThresh), InflateRadius(_InflateRadius)
    {
    }

    RRTConfig(const RRTConfig& _config):
        Euclidean(_config.Euclidean), OccupyThresh(_config.OccupyThresh), 
        InflateRadius(_config.InflateRadius),goal_sample_rate(_config.goal_sample_rate),
        extend_dis(_config.extend_dis),rand_area(_config.rand_area),
        max_iteration(_config.max_iteration){}

     
};

class RRT{
public:
    void InitRRT(cv::Mat& _Map,pathplanning::RRTConfig& _config);
    void InitRRT(cv::Mat& Map,cv::Mat& Mask,pathplanning::RRTConfig& config);
    void PathPlanning(cv::Point& _startPoint,cv::Point& _targetPoint,std::deque<cv::Point>& path);
    void MapProcess(cv::Mat& Mask);
    Node* FindPath();
    void GetPath(pathplanning::Node *TailNode, std::deque<cv::Point> &path);
    void DrawPath(cv::Mat& _Map, std::deque<cv::Point>& path, cv::InputArray Mask, cv::Scalar color,
        int thickness, cv::Scalar maskcolor);

    cv::Point sample_free();
    Node* get_nearest_node_idx(std::vector<Node*>&_node_list,cv::Point& _rand_point);
    cv::Point extend_point(Node& _nearest_node,cv::Point& _rand_point);
    bool inside_extend_area(cv::Point& _new_point);
    bool obstacle_free(cv::Point&new_point,Node& nearst_Node);
    double distence_to_target(cv::Point& _new_point);


    std::vector<Node*> node_list;

private:
    cv::Mat Map;
    cv::Mat LabelMap;
    cv::Point startPoint,targetPoint;
    RRTConfig config;
    

};


}




#endif 