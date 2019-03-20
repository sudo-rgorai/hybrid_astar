#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>

#include "../src/Planner.cpp"
#include "../include/GUI.hpp"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h" 
#include "geometry_msgs/PoseArray.h" 
#include "geometry_msgs/PoseStamped.h" 
#include "geometry_msgs/TransformStamped.h" 

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "tf2/convert.h" 
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace cv;

typedef struct _Quaternion
{
    float x;
    float y;
    float z;
    float w;
}Quaternion;


State start, dest;
tf2_ros::Buffer tfBuffer;

nav_msgs::OccupancyGrid obs_grid;
pair< vector< vector< bool > >, double > obs_map;

bool map_ch = false;
bool dest_ch = false;
bool start_ch = false;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // cout<<"Inside mapCallback"<<endl;
    map_ch = true;
    obs_grid=*msg;
    
    obs_map.first.clear();
    // Convention of data in OccupancyGrid - height:= cols and width:= rows
    obs_map.first.resize(obs_grid.info.width,vector<bool>(obs_grid.info.height));
    
    for(int i=0; i<obs_grid.info.width; i++) 
        for(int j=0; j < obs_grid.info.height; j++)
            obs_map.first[i][j] = (obs_grid.data[j*obs_grid.info.height+i]>= 90 || obs_grid.data[j*obs_grid.info.height+i]==-1); 

    obs_map.second = obs_grid.info.resolution;

    // cout<<"Map "<<obs_grid.info.width<<" "<<obs_grid.info.height<<endl;

}

// Used to locate the current position of vehicle
void odomCallback(const nav_msgs::Odometry& odom_msg) 
{
    // cout<<"Inside OdomCallback"<<endl;
    start_ch = true;

    geometry_msgs::PoseStamped begin;
    begin.pose = odom_msg.pose.pose;

    geometry_msgs::PoseStamped  trans_begin;
    geometry_msgs::TransformStamped trans_msg;
    
    try{
        trans_msg = tfBuffer.lookupTransform("hybrid_astar", "map",ros::Time(0), ros::Duration(10));
        tf2::doTransform(begin,trans_begin,trans_msg);
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
    }

    start.x = trans_begin.pose.position.x ;
    start.y = trans_begin.pose.position.y ;

    tf::Quaternion q(trans_begin.pose.orientation.x, trans_begin.pose.orientation.y, trans_begin.pose.orientation.z, trans_begin.pose.orientation.w);
    tf::Matrix3x3 m(q);
    
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    start.theta = fmod(yaw+2*M_PI,2*M_PI);
    
}

// Used to accept the goal position of vehicle
void goalCallback(const geometry_msgs::PoseStamped&  goal)
{
    // cout<<"Inside goalCallback"<<endl;
    dest_ch = true;
    
    geometry_msgs::PoseStamped  trans_goal;
    geometry_msgs::TransformStamped trans_msg;
    
    try{
        trans_msg = tfBuffer.lookupTransform("hybrid_astar", "odom",ros::Time(0));
        tf2::doTransform(goal,trans_goal,trans_msg);
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
    }
    
    dest.x= trans_goal.pose.position.x ;
    dest.y= trans_goal.pose.position.y ;
        
    tf::Quaternion q(trans_goal.pose.orientation.x, trans_goal.pose.orientation.y, trans_goal.pose.orientation.z, trans_goal.pose.orientation.w);
    tf::Matrix3x3 m(q);
    
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    dest.theta=fmod(yaw+2*M_PI,2*M_PI);
}

Quaternion toQuaternion(double M_PItch, double roll, double yaw)
{
    Quaternion q;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(M_PItch * 0.5);
    double sp = sin(M_PItch * 0.5);

    q.w = cy * cr * cp + sy * sr * sp;
    q.x = cy * sr * cp - sy * cr * sp;
    q.y = cy * cr * sp + sy * sr * cp;
    q.z = sy * cr * cp - cy * sr * sp;
    return q;
}


int main(int argc,char **argv)
{ 
    bool DEBUG = true;
    
    int rows = 200, cols = 200;
    float scale = 4;
    
    ros::init(argc,argv,"hybrid_astar");
    ros::NodeHandle nh;
    
    ros::Subscriber map  = nh.subscribe("/costmap_node/costmap/costmap",10,&mapCallback);   
    ros::Subscriber odom  = nh.subscribe("/base_pose_ground_truth",10,&odomCallback);
    ros::Subscriber goal  = nh.subscribe("/move_base_simple/goal",10,&goalCallback);

    ros::Publisher  pub = nh.advertise<nav_msgs::Path>("/astroid_path", 10);
    tf2_ros::TransformListener listener(tfBuffer);

    ros::Rate rate(1);

    Planner astar;
    Vehicle car;
    
    while(ros::ok())
    {
        
        nav_msgs::Path path_pub; 
        path_pub.header.frame_id = "/map";

        while( !start_ch )
        {
            cout<<"Waiting for Start "<<endl;
            ros::spinOnce();
        }
        cout<<"Starting Received : "<<start.x<<" "<<start.y<<" "<<start.theta<<endl;

        while(!dest_ch)
        {
            cout<<"Waiting for Goal "<<endl;
            ros::spinOnce();
        }
        cout<<"Destination Received : "<<dest.x<<" "<<dest.y<<" "<<dest.theta<<endl;

        while( !map_ch )
        {
            cout<<"Waiting for Map "<<endl;
            ros::spinOnce();
        }

        rows = obs_map.first.size();
        cols = obs_map.first[0].size();
        GUI display(rows,cols,scale);

        clock_t start_time=clock();
        vector<State> path = astar.plan(start, dest, car, obs_map, display, rows, cols);
        clock_t end_time=clock();

        geometry_msgs::PoseStamped  trans_pose;
        geometry_msgs::TransformStamped trans_msg;
    
        try{
            trans_msg = tfBuffer.lookupTransform("map", "hybrid_astar",ros::Time(0), ros::Duration(10));
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
        }

        vector<State>::iterator ptr;
        for (ptr = path.begin(); ptr != path.end(); ptr++) 
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = (*ptr).x;
            pose.pose.position.y = (*ptr).y;
            pose.pose.position.z = 0;

            Quaternion myQuaternion = toQuaternion(0,0,(*ptr).theta);

            pose.pose.orientation.x = myQuaternion.x;
            pose.pose.orientation.y = myQuaternion.y;
            pose.pose.orientation.z = myQuaternion.z;
            pose.pose.orientation.w = myQuaternion.w;
           
            pose.header.frame_id = "hybrid_astar";
            pose.header.stamp = ros::Time::now();
            
            tf2::doTransform(pose,trans_pose,trans_msg);
            path_pub.poses.push_back(trans_pose);
        }
        cout<<"Total time taken: "<<(double)(end_time-start_time)/CLOCKS_PER_SEC<<endl;
        cout<<"Got path of length "<<path.size()<<endl<<endl;
        astar.path.clear();
       
        path_pub.header.stamp = ros::Time::now();
        pub.publish(path_pub);
        rate.sleep();
        exit(0);

    }

}