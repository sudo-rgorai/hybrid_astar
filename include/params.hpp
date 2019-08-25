#ifndef PARAMS
#define PARAMS
#include<iostream>
using namespace std;
#include<queue>
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/core/core.hpp"
#include"opencv2/imgproc/imgproc.hpp"
using namespace cv;


struct Node1{ //Used to find nearest obstacle distance and voronoi edge distance for each point 
	int x,y;
	int color_code; 
	float cost;

	Node1( int x, int y, int color_code,float cost) : x(x),y(y),color_code(color_code), cost(cost)
	{

	}

};

struct Compare { 
    bool operator()(Node1 const& p1, Node1 const& p2) 
    { 
        return (p1.cost > p2.cost); 
    } 
};

struct node1{  //Used to identify no of obstacles and their borders
	int x,y;
};
priority_queue<Node1, vector<Node1>, Compare> main_q;  //Used to find nearest obstacle distance for each point
priority_queue<Node1, vector<Node1>, Compare> main_q2;   //Used to find nearest voronoi edge distance for each point

queue <node1> myq;  //Used to identify no of obstacles and their borders
Mat cost_image;  //Used to store nearest obstacle distance for each point
Mat voronoi_cost_image;  //Used to store nearest voronoi edge distance for each point
Mat voronoi_edges;  //Used to represent voronoi edges
Mat common;
Mat final;
int max_obs_dist = 250;  //Parameteers
int alpha = 250;

#endif