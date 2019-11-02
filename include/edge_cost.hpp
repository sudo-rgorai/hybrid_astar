#ifndef EDGE_COST
#define EDGE_COST
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/core/core.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include<iostream>
using namespace cv;
using namespace std;
#include<queue>
#include<math.h>

int isvalid2(Mat input,int i, int j);

Mat find_edge_cost(Mat input)
{	
	Node1 temp_node1=Node1(0,0,0,0),current_node1=Node1(0,0,0,0);
	Mat occupancy_grid=Mat(input.rows,input.cols,CV_8UC1,Scalar(0));
	int i,j;
	voronoi_cost_image = Mat(input.rows,input.cols,CV_8UC1,Scalar(255));
	for(i=0;i<input.rows;i++) for(int j=0;j<input.cols;j++) voronoi_cost_image.at<uchar>(i,j) = 254;
	if(main_q2.empty())
	{
		return input;
	}
	current_node1=main_q2.top();
	i=current_node1.x;
	j=current_node1.y;
	int l,m;
	while(!main_q2.empty())
	{
			if(!main_q2.empty())	
			{
				current_node1=main_q2.top();
				i=current_node1.x;j=current_node1.y;
				main_q2.pop();
				if(occupancy_grid.at<uchar>(i,j)==255) continue;
				occupancy_grid.at<uchar>(i,j)=255;
				
				if(current_node1.cost<255) voronoi_cost_image.at<uchar>(i,j)=(int)current_node1.cost;
				else voronoi_cost_image.at<uchar>(i,j) = 254;
			}    
			for(l=i-1;l<=i+1;l++)
			{
				for(m=j-1;m<=j+1;m++)
				{	
					if(isvalid2(input,l,m)&&occupancy_grid.at<uchar>(l,m)==0)
					{
						temp_node1=Node1(l,m,current_node1.color_code,current_node1.cost+1.0+0.414*(abs(l-i+m-j+1)%2));
						main_q2.push(temp_node1);
					}

				}
			}
			
	}
	return voronoi_cost_image;

}



int isvalid2(Mat input,int x, int y)
{
	if(x<0||y<0||x>input.rows-1||y>input.cols-1){
		
	 	return 0;
	}
	else return 1;
}

#endif