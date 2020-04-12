#ifndef BFS
#define BFS
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/core/core.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include<iostream>
using namespace cv;
using namespace std;

#include<queue>

void bfs(Mat obstacle_map, Mat occupancy_grid, int i,int j,int count);
int isvalid(Mat obstacle_map,int i, int j);

Node1 temp_node1(0,0,0,0);
	
void main_bfs(Mat input_obstacle_map)     //Used to identify individual obstacles and find their edges
{
	int l,m;
	Mat obstacle_map(input_obstacle_map.rows,input_obstacle_map.cols,CV_8UC1,Scalar(0));	
	for(l=0;l<obstacle_map.rows;l++)
	{
		for(m=0;m<obstacle_map.cols;m++)
		{
			if(input_obstacle_map.at<uchar>(l,m)<150)
				obstacle_map.at<uchar>(l,m)=0;
			else
				obstacle_map.at<uchar>(l,m)=255;
		}
	}

	Mat occupancy_grid=obstacle_map.clone();
	int i,j,k;
	node1 temp;
	int count=0;

	for(l=0;l<obstacle_map.rows-1;l++)
	{
		for(m=0;m<obstacle_map.cols-1;m++)
		{	
			
			if(occupancy_grid.at<uchar>(l,m)==255)
			{
				temp.x=l;
				temp.y=m;
				temp_node1=Node1(temp.x,temp.y,count+1,0);
				main_q.push(temp_node1);
				myq.push(temp);
				occupancy_grid.at<uchar>(l,m)=0;
				bfs(obstacle_map,occupancy_grid,l,m,count);
				count++; 
			}
		}
	}
	cout << "**************No of obstacles : \n\n\n\n\n\n\n\n\n\n" << count;
}

void bfs(Mat obstacle_map,Mat occupancy_grid,int i,int j,int count)   //Used to apply bfs to travers whole of the obstacle edge
{	
	node1 temp;
	int l,m;
	while(!myq.empty())
	{
		for(l=i-1;l<=i+1;l++)
		{
			for(m=j-1;m<=j+1;m++)
			{
				if(isvalid(obstacle_map,l,m)&&occupancy_grid.at<uchar>(l,m)==255)
				{
					occupancy_grid.at<uchar>(l,m)=0;
					common.at<uchar>(l,m)=255;
					temp.x=l;
					temp.y=m;
					temp_node1=Node1(l,m,count+1,0.0);
					main_q.push(temp_node1);
					myq.push(temp);
				}
			}
		}
		myq.pop();
		if(!myq.empty())	
		{
			temp=myq.front();
			i=temp.x;j=temp.y;
		}
		                                
	}
	
}

/*Point bfs2(Mat obstacle_map,Mat occupancy_grid,Mat obstacles_numbered,int i,int j,int count)   //Used to apply bfs to travers whole of the obstacle edge
{	
	node1 temp;
	int l,m;
	Point p;
	p.x = 0;p.y=0;int no=0;
	
	while(!myq.empty())
	{
		for(l=i-1;l<=i+1;l++)
		{
			for(m=j-1;m<=j+1;m++)
			{
				if(isvalid(obstacle_map,l,m)&&occupancy_grid.at<uchar>(l,m)==255)
				{
					p.x += i;
					p.y += j;
					no++;
					
					occupancy_grid.at<uchar>(l,m)=0;
					obstacles_numbered.at<uchar>(l,m)=count;
					temp.x=l;
					temp.y=m;
					temp_node1=Node1(l,m,count+1,0.0);
					main_q.push(temp_node1);
					myq.push(temp);
				}
			}
		}
		myq.pop();
		if(!myq.empty())	
		{
			temp=myq.front();
			i=temp.x;j=temp.y;
		}                                
	}
	p.x/=no;
	p.y/=no;
	return p;
}


void number_obstacles(Mat input_obstacle_map)     //Used to identify individual obstacles and find their edges
{
	int l,m;
	Mat obstacle_map(input_obstacle_map.rows,input_obstacle_map.cols,CV_8UC1,Scalar(0));	
	Mat obstacles_numbered(input_obstacle_map.rows,input_obstacle_map.cols,CV_8UC1,Scalar(255));	
	for(l=0;l<obstacle_map.rows;l++)
	{
		for(m=0;m<obstacle_map.cols;m++)
		{
			if(input_obstacle_map.at<uchar>(l,m)<150)
				obstacle_map.at<uchar>(l,m)=0;
			else
				obstacle_map.at<uchar>(l,m)=255;
		}
	}
	Mat occupancy_grid=obstacle_map.clone();
	int i,j,k;
	node1 temp;
	int count=0;
	int unassigned_colour=0;
	for(l=0;l<obstacle_map.rows;l++)
	{
		for(m=0;m<obstacle_map.cols;m++)
		{	
			
			if(occupancy_grid.at<uchar>(l,m)==255)
			{
				temp.x=l;
				temp.y=m;
				temp_node1=Node1(temp.x,temp.y,count+1,0);
				main_q.push(temp_node1);
				myq.push(temp);
				occupancy_grid.at<uchar>(l,m)=0;
				Point temp;
				temp = bfs2(obstacle_map,occupancy_grid,obstacles_numbered,l,m,count);
				obstacle_centres[count] = temp;
				float min_dist = 1000;int min_dist_index=0;
				for(int i=0;i<prev_count;i++){
					if(sqrt((temp.x-prev_obstacle_centres[i].x)*(temp.x-prev_obstacle_centres[i].x) + (temp.y-prev_obstacle_centres[i].y)*(temp.y-prev_obstacle_centres[i].y)) < min_dist) 
					{
						min_dist = sqrt((temp.x-prev_obstacle_centres[i].x)*(temp.x-prev_obstacle_centres[i].x) + (temp.y-prev_obstacle_centres[i].y)*(temp.y-prev_obstacle_centres[i].y));
						min_dist_index = i; 
					}
				}
				if(min_dist < tolerance){
					colour_code[count] = prev_colour_code[min_dist_index];
					colour_code[count] = colour_code[count] % 10;
					unassigned_colour = (colour_code[count] + 2)%10; 
				}  
				else {
					colour_code[count] = (unassigned_colour++)%10;
				}
				count++; 
			}
		}
	}
	Mat coloured_obstacles(input_obstacle_map.rows,input_obstacle_map.cols,CV_8UC3,Scalar(255,255,255));
	int prevx = 0;int value = 0;int no=0;int prev_obs_x = 0;
	for(i=0;i<obstacle_map.rows-1;i++)
	{
		for(j=0;j<obstacle_map.cols-1;j++)
		{
			
			if((int)obstacles_numbered.at<uchar>(i,j)==255)continue;
			int temp = colour_code[(int)obstacles_numbered.at<uchar>(i,j)]%10;
			int x = obstacle_centres[(int)obstacles_numbered.at<uchar>(i,j)].x - obstacle_map.rows/2;
			int y = obstacle_centres[(int)obstacles_numbered.at<uchar>(i,j)].y - obstacle_map.cols/2;
			if(x>0) {
				if((x-prevx)>tolerance) {
					cout << "Layer " << value/100 + 1 << ": " << no <<endl; 
					value += 100;
					prevx = x;
				}
				else if (x<prevx){
					if((value-100)<255) obstacles_numbered.at<uchar>(obstacle_map.rows-1-i,obstacle_map.cols-1-j) = value-100;
					else obstacles_numbered.at<uchar>(obstacle_map.rows-1-i,obstacle_map.cols-1-j)=255;
					obstacles_numbered.at<uchar>(i,j) = 255;
					continue;
				} 
				if(x > prev_obs_x) {
					prev_obs_x = x;
					no++;
				}
				 
				
				if(value<255) obstacles_numbered.at<uchar>(obstacle_map.rows-1-i,obstacle_map.cols-1-j) = value;
				else obstacles_numbered.at<uchar>(obstacle_map.rows-1-i,obstacle_map.cols-1-j)=255;
				obstacles_numbered.at<uchar>(i,j) = 255;
			}
			else obstacles_numbered.at<uchar>(i,j) = 255;
			if(temp==0){
				coloured_obstacles.at<Vec3b>(i,j)[0] = 255;
				coloured_obstacles.at<Vec3b>(i,j)[1] = 0;
				coloured_obstacles.at<Vec3b>(i,j)[2] = 0;
			}
			if(temp==1){
				coloured_obstacles.at<Vec3b>(i,j)[0] = 0;
				coloured_obstacles.at<Vec3b>(i,j)[1] = 255;
				coloured_obstacles.at<Vec3b>(i,j)[2] = 0;
			}
			if(temp==2){
				coloured_obstacles.at<Vec3b>(i,j)[0] = 0;
				coloured_obstacles.at<Vec3b>(i,j)[1] = 0;
				coloured_obstacles.at<Vec3b>(i,j)[2] = 255;
			}
			if(temp==3){
				coloured_obstacles.at<Vec3b>(i,j)[0] = 255;
				coloured_obstacles.at<Vec3b>(i,j)[1] = 255;
				coloured_obstacles.at<Vec3b>(i,j)[2] = 0;
			}
			if(temp==4){
				coloured_obstacles.at<Vec3b>(i,j)[0] = 255;
				coloured_obstacles.at<Vec3b>(i,j)[1] = 0;
				coloured_obstacles.at<Vec3b>(i,j)[2] = 255;
			}
			if(temp==5){
				coloured_obstacles.at<Vec3b>(i,j)[0] = 0;
				coloured_obstacles.at<Vec3b>(i,j)[1] = 255;
				coloured_obstacles.at<Vec3b>(i,j)[2] = 255;
			}
			if(temp==6){
				coloured_obstacles.at<Vec3b>(i,j)[0] = 255;
				coloured_obstacles.at<Vec3b>(i,j)[1] = 127;
				coloured_obstacles.at<Vec3b>(i,j)[2] = 0;
			}
			if(temp==7){
				coloured_obstacles.at<Vec3b>(i,j)[0] = 255;
				coloured_obstacles.at<Vec3b>(i,j)[1] = 0;
				coloured_obstacles.at<Vec3b>(i,j)[2] = 127;
			}
			if(temp==8){
				coloured_obstacles.at<Vec3b>(i,j)[0] = 0;
				coloured_obstacles.at<Vec3b>(i,j)[1] = 127;
				coloured_obstacles.at<Vec3b>(i,j)[2] = 255;
			}
			if(temp==9){
				coloured_obstacles.at<Vec3b>(i,j)[0] = 127;
				coloured_obstacles.at<Vec3b>(i,j)[1] = 255;
				coloured_obstacles.at<Vec3b>(i,j)[2] = 0;
			}
		}
	}
	arrowedLine(obstacles_numbered, Point(obstacles_numbered.rows/2,obstacles_numbered.cols/2), Point(obstacles_numbered.rows/2,obstacles_numbered.cols/2-10), Scalar(0), 2, 8, 0, 0.2);
	prev_count = count;
	imshow("Output focused on closer obstacles",obstacles_numbered);
	
	waitKey(10);
	for(int i=0;i<prev_count;i++) {
		prev_colour_code[i] = colour_code[i];
		prev_obstacle_centres[i] = obstacle_centres[i];
	}
	cout << "chala gya be" <<endl;
	return;	
}
*/

int isvalid(Mat obstacle_map,int x, int y)
{
	if(x<0||y<0||x>obstacle_map.rows-1||y>obstacle_map.cols-1)
	{
		return 0;
	}
	else return 1;
}

#endif