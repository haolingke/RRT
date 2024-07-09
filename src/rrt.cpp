// created by lililinghao on 24-07-5

#include "RRT.h"
#include <ros/ros.h>

namespace pathplanning{

void RRT::InitRRT(cv::Mat& _Map,pathplanning::RRTConfig& _config){
    cv::Mat Mask;
    InitRRT(_Map, Mask, _config);
}

void RRT::InitRRT(cv::Mat& _Map,cv::Mat& Mask,pathplanning::RRTConfig& _config){
    Map = _Map;
    config = _config;
    config.rand_area = _Map.size().height>_Map.size().width?_Map.size().height:_Map.size().width;   
    MapProcess(Mask);
}

void RRT::PathPlanning(cv::Point& _startPoint,cv::Point& _targetPoint,std::deque<cv::Point>& path)
{
    //get variables
    startPoint = _startPoint;
    targetPoint = _targetPoint;

    //Path Planning
    Node* TailNode = FindPath();
    GetPath(TailNode,path);


}

void RRT::DrawPath(cv::Mat& _Map, std::deque<cv::Point>& path, cv::InputArray Mask, cv::Scalar color,
        int thickness, cv::Scalar maskcolor)
{
    if(path.empty())
    {
        std::cout << "Path is empty!" << std::endl;
        return;
    }
    _Map.setTo(maskcolor, Mask);
    // for(auto it:path)
    for(const auto &it:path) //modified by llh
    {
        cv::rectangle(_Map, it, it, color, thickness);
    }
}

void RRT::MapProcess(cv::Mat& Mask)
{
    int width = Map.cols;
    int height = Map.rows;
    cv::Mat _Map = Map.clone();

    // Transform RGB to gray image
    if(_Map.channels() == 3)
    {
        cv::cvtColor(_Map.clone(), _Map, cv::COLOR_BGR2GRAY);
    }

    // Binarize
    if(config.OccupyThresh < 0)
    {
        cv::threshold(_Map.clone(), _Map, 0, 255, cv::THRESH_OTSU);
    } else
    {
        cv::threshold(_Map.clone(), _Map, config.OccupyThresh, 255, cv::THRESH_BINARY);
    }
    //膨胀（膨胀可以消除图像中的小黑点噪声，这些噪声在二值化后会显现为前景中的小孔）
    cv::Mat src = _Map.clone();
    if(config.InflateRadius > 0)
    {
        cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * config.InflateRadius, 2 * config.InflateRadius));
        //得到一个(2 * config.InflateRadius, 2 * config.InflateRadius)的矩形结构元素
        cv::erode(src, _Map, se);
    }

    //Get mask
    cv::bitwise_xor(src, _Map, Mask);  //按位异或

    // Initial LabelMap
    LabelMap = cv::Mat::zeros(height, width, CV_8UC1);
    for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
            if(_Map.at<uchar>(y, x) == 0)
            {
                LabelMap.at<uchar>(y, x) = obstacle;
            }
            else
            {
                LabelMap.at<uchar>(y, x) = free;
            }
        }
    }
}

Node* RRT::FindPath()
{
    ROS_INFO("start findPath");
    int width = Map.cols;
    int height = Map.rows;
    cv::Mat _LabelMap = LabelMap.clone();

    //初始化起始节点
    Node* StartNode = new Node(startPoint,nullptr);
    node_list.clear();
    node_list.push_back(StartNode);

    for(int i=0;i<config.max_iteration;i++)
    {
        //得到随机目标点
        cv::Point rand_point = sample_free();
        Node* nearest_iter = get_nearest_node_idx(node_list,rand_point);
        Node nearst_Node = *nearest_iter;
        //ROS_INFO("nearset point x=%d, y=%d ",nearest_iter->point.x,nearest_iter->point.y);
        cv::Point new_point = extend_point(nearst_Node ,rand_point);
        //如果new_point在target_point附近，将target_point设为new_point
        if(distence_to_target(new_point) <= config.extend_dis){
            new_point = targetPoint;
        }
        //判断是否在可行区域内
        if(inside_extend_area(new_point) && obstacle_free(new_point,nearst_Node)){
            Node* new_Node = new Node(new_point, nearest_iter);
            node_list.push_back(new_Node);
            //ROS_INFO("find a new node:x=%d ,y=%d",new_Node.point.x,new_Node.point.y);
        }

        if(node_list.back()->point == targetPoint)
        {
            break;
        }
    int node_list_length = node_list.size();
    //ROS_INFO("%d",node_list_length);
        //Node new_node(expendNode(nearst_Node),nearest_iter);
    }

    Node* nowptr = node_list.back();
    for(std::vector<Node*>::iterator debug_ite = node_list.begin();debug_ite != node_list.end();debug_ite++)
    {
        ROS_INFO("%p",(*debug_ite)->parent);
    }


    return node_list.back();

}

void RRT::GetPath(Node *TailNode, std::deque<cv::Point> &path)
{   Node *nowptr = TailNode;
    try
    {
        path.clear();
        // Node *nowptr = TailNode;
        int flag = 0;
        while(nowptr != NULL){
            path.push_front(nowptr->point);
            ROS_INFO("path point x=%d,y=%d",nowptr->point.x,nowptr->point.y);
            ROS_INFO("Current Node: %p, Parent Node: %p", nowptr, nowptr->parent);
            std::cout<<flag++<<"  "<<node_list.size()<<std::endl;
            nowptr = nowptr->parent;
            if(nowptr == *node_list.begin()){
                ROS_INFO("return to startpoint");
                break;
            }
        }
    }catch(const std::exception &e) 
    {
        ROS_ERROR("Exception caught in GetPath: %s nowptr->parent:%p", e.what(),nowptr->parent);
    }
}


cv::Point RRT::sample_free()
{
    // ROS_INFO("start sample");
    std::random_device rd;
    std::uniform_int_distribution<int> target_probility_dist(0,1000);
    std::uniform_int_distribution<int> rand_sample_dist(0,config.rand_area);
    cv::Point rand_point;
    if(target_probility_dist(rd) <= 1000*config.goal_sample_rate){
        rand_point = targetPoint;
    }
    else{
        rand_point.x = rand_sample_dist(rd); 
        rand_point.y = rand_sample_dist(rd);
    }
    //ROS_INFO("new rand node:x=%d ,y=%d",rand_point.x,rand_point.y);
    //ROS_INFO("sample finished,x=%s,y=%s",rand_point.x,rand_point.y);
    return rand_point;
}

Node* RRT::get_nearest_node_idx(std::vector<Node*>&_node_list,cv::Point& _rand_point)
{
    std::vector<Node*>::iterator nearest_node_idx;
    int shortest_twoPoint_distance = 100000;
    for(std::vector<Node*>::iterator node_list_iter = _node_list.begin();node_list_iter != _node_list.end();node_list_iter++)
    {
        double twoPoint_distance = abs((*node_list_iter)->point.x - _rand_point.x)+
        abs((*node_list_iter)->point.y - _rand_point.y);
        nearest_node_idx = twoPoint_distance < shortest_twoPoint_distance ? node_list_iter : nearest_node_idx;
    }
    return *nearest_node_idx;
}

cv::Point RRT::extend_point(Node& _nearest_node,cv::Point& _rand_point)
{
    double dx = _rand_point.x - _nearest_node.point.x;
    double dy = _rand_point.y - _nearest_node.point.y;
    double extend_distance = std::sqrt(dx*dx+dy*dy);
    
    // double theta = _nearest_node.point.x !=_rand_point.x ? std::atan2(dy,dx) : M_PI/2;
    //atan返回的是-pi/2~pi/2，不适合这里
    double new_x,new_y;
    if(extend_distance < config.extend_dis){
        new_x = _rand_point.x;
        new_y = _rand_point.y;
    }
    else{
        new_x = _nearest_node.point.x + dx/extend_distance*config.extend_dis;
        new_y = _nearest_node.point.y + dy/extend_distance*config.extend_dis;
    }
    cv::Point new_point(floor(new_x),floor(new_y));
    return new_point;
}

bool RRT::inside_extend_area(cv::Point& _new_point)
{
    if(config.rand_area == 0){return true;}
    else if(_new_point.x<config.rand_area && _new_point.x>0
    &&_new_point.y<config.rand_area && _new_point.y>0
    ){return true;}
    else{return false;}
}

bool RRT::obstacle_free(cv::Point&_new_point,Node& _nearst_Node){
    //没想好怎么写，暂时只考虑终点不碰撞
    // return LabelMap.at<uchar>(_new_point.x,_new_point.y == free);
    return true;
}

double RRT::distence_to_target(cv::Point& _new_point){
    return abs(_new_point.x-targetPoint.x)+abs(_new_point.y-targetPoint.y);
}

} // namespace pathplannig




