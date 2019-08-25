#include"../include/params.hpp"
#include "../include/Planner.hpp"
#include "../include/GUI.hpp"
#include "../include/voronoi.hpp"

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h" 
#include "geometry_msgs/Pose.h" 
#include "geometry_msgs/PoseStamped.h" 
#include "geometry_msgs/TransformStamped.h" 
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/core/core.hpp"
#include"opencv2/imgproc/imgproc.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include <tf/transform_datatypes.h>
#include "tf2/convert.h" 
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

typedef struct _Quaternion
{
    float x;
    float y;
    float z;
    float w;
}Quaternion;

Mat costmap;

class ROSInterface
{
    public:
    int distThreshold = 5;

    State curr;
	State start;
    State destination;
    
    int** map;
    float map_origin_x;
    float map_origin_y;
    int map_grid_x;
    int map_grid_y;
    float map_grid_resolution;

    bool got_map;
    bool got_start;
    bool got_destination;

    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener;

    ros::Subscriber curr_sub;
    ros::Subscriber start_sub;
    ros::Subscriber destination_sub;
    ros::Subscriber map_sub;
    ros::Publisher path_pub;

    void repeatedDestination(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        if(!got_destination)
            return;

        cout<<"Getting current position"<<endl;

        curr.x = odom_msg->pose.pose.position.x;
        curr.y = odom_msg->pose.pose.position.y;

        tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        curr.theta = fmod(yaw+2*M_PI,2*M_PI);

        if( sqrt((curr.x-destination.x)*(curr.x-destination.x) + (curr.y-destination.y)*(curr.y-destination.y))< distThreshold)
        {
            got_destination = false;
            cout<<"***************************Waypoint Reached****************************************"<<endl;
        }    

        return;
    }

    void startCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) 
    {
        if(got_start)
            return;
        
        cout<<"Getting start position."<<endl;

        start.x = odom_msg->pose.pose.position.x;
        start.y = odom_msg->pose.pose.position.y;

        tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        start.theta = fmod(yaw+2*M_PI,2*M_PI);
        got_start = true;
        return;
    }

    void destinationCallback(const geometry_msgs::PoseStamped& goal_msg)
    {
        if(got_destination)
            return;

        cout<<"Getting destination"<<endl;
        
        geometry_msgs::PoseStamped transformed_goal;
        geometry_msgs::TransformStamped transform_msg;
        
        try{
            transform_msg = tfBuffer.lookupTransform("map", goal_msg.header.frame_id, ros::Time(0));
            tf2::doTransform(goal_msg, transformed_goal, transform_msg);
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
        }
        
        destination.x= transformed_goal.pose.position.x;
        destination.y= transformed_goal.pose.position.y;
            
        tf::Quaternion q(transformed_goal.pose.orientation.x, transformed_goal.pose.orientation.y, transformed_goal.pose.orientation.z, transformed_goal.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        destination.theta=fmod(yaw+2*M_PI,2*M_PI);

        got_destination = true;
        return;
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        if(got_map)
            return;

        cout<<"Getting map"<<endl;

        map_origin_x = msg->info.origin.position.x;
        map_origin_y = msg->info.origin.position.y;
        map_grid_x = msg->info.width;
        map_grid_y = msg->info.height;
        map_grid_resolution = msg->info.resolution;
        costmap = Mat(map_grid_x,map_grid_y,CV_8UC1,Scalar(0));
        voronoi(costmap);

        //costmap is stored in row-major format
        for(int j=0; j<map_grid_y; j++)
            for(int i=0; i<map_grid_x; i++)
            {
                map[i][j] = msg->data[j*200 + i] != 0 ? 1 : 0;
                costmap.at<uchar>(i,j) = map[i][j];
            }
        got_map = true;
        return;
    }

    void reset()
    {
        got_start = false;
        got_destination = false;
        got_map = false;

        return;
    }

    void transform_start_and_destination()
    {
        start.x = start.x - map_origin_x;
        start.y = start.y - map_origin_y;

        destination.x = destination.x - map_origin_x;
        destination.y = destination.y - map_origin_y;

    }

    void transform_back_destination()
    {
        destination.x = destination.x + map_origin_x;
        destination.y = destination.y + map_origin_y;
    }

    Quaternion toQuaternion(double pitch, double roll, double yaw)
    {
        Quaternion q;

        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);

        q.w = cy * cr * cp + sy * sr * sp;
        q.x = cy * sr * cp - sy * cr * sp;
        q.y = cy * cr * sp + sy * sr * cp;
        q.z = sy * cr * cp - cy * sr * sp;
        return q;
    }

    nav_msgs::Path convert_to_path_msg(vector<State> path)
    {
        string trans_frame = "map";
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = trans_frame;

        geometry_msgs::PoseStamped pose, transformed_pose;
        geometry_msgs::TransformStamped transform_msg;

        try{
            transform_msg = tfBuffer.lookupTransform(trans_frame, "map", ros::Time(0));
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
        }

        for(vector<State>::iterator ptr = path.begin(); ptr != path.end(); ptr++)
        {

            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time::now();

            pose.pose.position.x = (*ptr).x + map_origin_x;
            pose.pose.position.y = (*ptr).y + map_origin_y;
            pose.pose.position.z = 0;

            Quaternion myQuaternion = toQuaternion(0,0,(*ptr).theta);

            pose.pose.orientation.x = myQuaternion.x;
            pose.pose.orientation.y = myQuaternion.y;
            pose.pose.orientation.z = myQuaternion.z;
            pose.pose.orientation.w = myQuaternion.w;

            tf2::doTransform(pose, transformed_pose, transform_msg);
            path_msg.poses.push_back(transformed_pose);
        }

        return path_msg;
    }

    void publish_path(nav_msgs::Path path_msg)
    {
        path_msg.header.stamp = ros::Time::now();
        path_pub.publish(path_msg);
        return;
    }

    ROSInterface(ros::NodeHandle nh) : listener(tfBuffer)
    {
        this->nh = nh;

        int* map_array=new int[200*200];
        this->map = new int*[200];
        for(int i=0;i<200;i++)
            (this->map)[i] = map_array + 200*i;

        curr_sub = nh.subscribe("/base_pose_ground_truth", 1, &ROSInterface::repeatedDestination, this);
        start_sub = nh.subscribe("/base_pose_ground_truth", 1, &ROSInterface::startCallback, this);
        destination_sub = nh.subscribe("/move_base_simple/goal", 1, &ROSInterface::destinationCallback, this);
        map_sub = nh.subscribe("/costmap_node/costmap/costmap", 1, &ROSInterface::mapCallback, this);
        path_pub = nh.advertise<nav_msgs::Path>("/astroid_path", 10);

        got_start = true;
        got_destination = true;
        got_map = true;
        
        return;
    }

    ~ROSInterface()
    {
        delete[] (this->map)[0];
        delete[] this->map;
        
        return;
    }
};

// This function can be called in main if path is to be planned once
void plan_once(ros::NodeHandle nh)
{ 
    ROSInterface interface(nh);
    interface.got_start = false;
    interface.got_destination = false;
    interface.got_map = false;

    ros::Rate wait_rate(20);
    while(ros::ok())
    {
        ros::spinOnce();

        if(interface.got_start && interface.got_destination && interface.got_map)
            break;

        wait_rate.sleep();
    }

    cout<<"Got start, destination and map."<<endl;

    interface.transform_start_and_destination();

    State start = interface.start;
    State destination = interface.destination;
    int** map = interface.map;
    int map_x = interface.map_grid_x * interface.map_grid_resolution;
    int map_y = interface.map_grid_y * interface.map_grid_resolution;
    float map_grid_resolution = interface.map_grid_resolution;
    float planner_grid_resolution = 0.5;

    Vehicle car;

    GUI display(map_x, map_y, 5); 
    display.draw_obstacles(map, map_grid_resolution);
    display.draw_car(start,car);
    display.draw_car(destination,car);
    display.show(1000);


    Planner astar(map_x, map_y, map_grid_resolution, planner_grid_resolution);
    vector<State> path = astar.plan(start, destination, car, map, display,final);

    // This has been done to increase the density of number of points on the path so that it can be tracked efficiently
    vector<State> path_expanded(2*path.size()-1);
    for (int i = 0; i < path.size(); ++i)
    {
        path_expanded[2*i] = path[i];
        if( 2*i+1 < path_expanded.size()) path_expanded[2*i+1] = { (path[i].x+path[i+1].x)/2,(path[i].y+path[i+1].y)/2,(path[i].theta+path[i+1].theta)/2,};
    }
    cout<<"Number of points in the generated path: "<<path_expanded.size()<<endl;


    for(int i=0;i<=path_expanded.size();i++)
    {
        display.draw_car(path_expanded[i], car);
        display.show(1);
    } 
    display.show(2000);

    while(ros::ok())
    {
        ros::spinOnce();

        nav_msgs::Path path_msg = interface.convert_to_path_msg(path_expanded);
        interface.publish_path(path_msg);

        wait_rate.sleep();
    }

    astar.path.clear();
    return;
}

// This function can be called in main to repeatedly planning path by publishing waypoints on /move_base_simple/goal
void plan_repeatedly(ros::NodeHandle nh)
{ 
    ROSInterface interface(nh);
    interface.got_start = true;
    interface.got_destination = false;
    interface.got_map = true;

    cout<<"Waiting for destination"<<endl;
    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();

        if(interface.got_destination)
            break;

        rate.sleep();
    }
    cout<<"Got destination."<<endl;

    int map_x, map_y;
    float map_grid_resolution;
    ros::param::get("/costmap_node/costmap/width", map_x);
    ros::param::get("/costmap_node/costmap/height", map_y);
    ros::param::get("/costmap_node/costmap/resolution", map_grid_resolution);

    float planner_grid_resolution = 0.5;

    Planner astar(map_x, map_y, map_grid_resolution, planner_grid_resolution);

    Vehicle car;
    GUI display(map_x, map_y, 5); 
    
    ros::Rate wait_rate(10);
    while(ros::ok())
    {
        interface.got_start = false;
        interface.got_map = false;

        clock_t start_time=clock();
        while(ros::ok())
        {
            ros::spinOnce();

            if(interface.got_start && interface.got_map && interface.got_destination)
                break;

            wait_rate.sleep();
        }
        clock_t end_time=clock();

        cout<<"Total time taken: "<<(double)(end_time-start_time)/CLOCKS_PER_SEC<<endl;
        interface.transform_start_and_destination();

        State start = interface.start;
        State destination = interface.destination;
        
        // map for every iteration retrived and used to plot obstacles on visualizer
        int** map = interface.map;

        // Planning path
        vector<State> path = astar.plan(start, destination, car, map, display,final);

        // This has been done to increase the density of number of points on the path so that it can be tracked efficiently
        vector<State> path_expanded(2*path.size()-1);
        for (int i = 0; i < path.size(); ++i)
        {
            path_expanded[2*i] = path[i];
            if( 2*i+1 < path_expanded.size()) path_expanded[2*i+1] = { (path[i].x+path[i+1].x)/2,(path[i].y+path[i+1].y)/2,(path[i].theta+path[i+1].theta)/2,};
        }
        cout<<"Number of points in the generated path: "<<path_expanded.size()<<endl;

        nav_msgs::Path path_msg = interface.convert_to_path_msg(path_expanded);
        interface.publish_path(path_msg);

        display.clear();
        astar.path.clear();
        path_expanded.clear();
        interface.transform_back_destination();
    }
    
    return;
}


int main(int argc,char **argv)
{ 
    ros::init(argc,argv,"hybrid_astar_node");
    ros::NodeHandle nh;
    Mat inp=imread("unnamed.jpg",0);
    plan_repeatedly(nh);
    
    return 0;
}

