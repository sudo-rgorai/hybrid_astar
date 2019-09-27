#include "../include/Planner.hpp"
#include <boost/heap/fibonacci_heap.hpp>

#define PI 3.14159

bool operator<(const State& a, const State& b)
{
	return (a.g + a.h)> (b.g + b.h);
}

Planner::Planner(int map_x, int map_y, float map_grid_resolution, float planner_grid_resolution)
{
	
	this->map_x = map_x;
	this->map_y = map_y;
	this->map_grid_resolution = map_grid_resolution;
	this->planner_grid_resolution = planner_grid_resolution;

	planner_grid_x = toInt(map_x/planner_grid_resolution);
	planner_grid_y = toInt(map_y/planner_grid_resolution);
	planner_grid_theta = 72;
	
	visited_state=new State**[planner_grid_x];
	for(int i=0;i<planner_grid_x;i++)
	{
		visited_state[i]=new State*[planner_grid_y];
		for(int j=0;j<planner_grid_y;j++)
			visited_state[i][j]=new State[72];
	}

	visited=new bool**[planner_grid_x];
	for(int i=0;i<planner_grid_x;i++)
	{
		visited[i]=new bool*[planner_grid_y];
		for(int j=0;j<planner_grid_y;j++)
			visited[i][j]=new bool[72];
	}
	
	return;
}

vector<State> Planner::plan(State start, State end, Vehicle car, int** obstacles, GUI display,Mat voronoi_values,Mat obs_dist) //voronoi_values matrix is the matrix storing the voronoi values at all the positions and obs_dist is the matrix storing the obstacle distances at each position
{
	bool DISPLAY_PATH = false;
	bool DISPLAY_SEARCH_TREE = false;

	clock_t map_init_start = clock();
	Map map(obstacles, map_x, map_y, map_grid_resolution, end, car);
	map_init_time = float(clock()-map_init_start)/CLOCKS_PER_SEC;

	if(DISPLAY_SEARCH_TREE)
    	display.draw_obstacles(obstacles, map_grid_resolution);

	clock_t dijkstra_start = clock();
	float dijkstra_grid_resolution = 1;
	Heuristic heuristic(map, dijkstra_grid_resolution, end, car);
	dijkstra_time = float(clock()-dijkstra_start)/CLOCKS_PER_SEC;
	
	for(int i=0; i < planner_grid_x; i++)
		for(int j=0; j < planner_grid_y; j++)
			for(int k=0; k < planner_grid_theta; k++ )
				visited[i][j][k]=false;
				

	// Hybrid Astar Openlist Initiates:
	priority_queue <State, vector<State>> pq;
	start.g = 0;
	start.h = heuristic.get_heuristic(start,voronoi_values);
	start.parent = NULL;
	pq.push(start);
	
	
	int count=0;
	while(!pq.empty())
	{

		State current=pq.top();
		pq.pop();

		// Angles are taken from 0-360 in steps of 5 so planner_grid_theta is 72 also 
		// theta attribute of State stores angle in radian form.
		int current_grid_x = roundDown(current.x/planner_grid_resolution);
		int current_grid_y = roundDown(current.y/planner_grid_resolution);
		int current_grid_theta = ((int)(current.theta*planner_grid_theta/(2*PI)))%planner_grid_theta;

		if( visited[current_grid_x][current_grid_y][current_grid_theta] )
			continue;

		visited[current_grid_x][current_grid_y][current_grid_theta] = true;
		visited_state[current_grid_x][current_grid_y][current_grid_theta] = current;	

		// Checks if it has reached the goal
		if(map.isReached(current))
		{	
			State temp=current;
			while( temp.parent != NULL )
			{
				path.push_back(temp);
				temp =*(temp.parent);
			}
			reverse(path.begin(), path.end());	

	        if(DISPLAY_PATH)
	        {
		     	display.clear();

	            display.draw_obstacles(obstacles, map_grid_resolution);
	            for(int i=0;i<=path.size();i++)
	            {
	                display.draw_car(path[i], car);
	                display.show(5);
	            } 
		     	display.show(1000);
		     	display.clear();
	        }		
			return path;
		}

		// This section finds the next states based on trajecory generation.
		vector<State> next=car.nextStates(&current);

		for(vector<State>::iterator it= next.begin(); it!=next.end();it++)
		{
			State nextS = *it;
			if( !map.isValid(nextS))
				continue;

			int next_grid_x = roundDown(nextS.x/planner_grid_resolution);;
			int next_grid_y = roundDown(nextS.y/planner_grid_resolution);
			int next_grid_theta = ((int)(nextS.theta*planner_grid_theta/(2*PI)))%planner_grid_theta;
				
			if( visited[next_grid_x][next_grid_y][next_grid_theta] )
				continue;
				
			if( !map.checkCollision(nextS) )   //change
			{
				if(DISPLAY_SEARCH_TREE)
				{
					display.draw_tree(current,nextS);
					display.show(5);					
				}

				it->parent = &(visited_state[current_grid_x][current_grid_y][current_grid_theta]);
				it->g = current.g+1;
				it->h = heuristic.get_heuristic(*it,voronoi_values);

				pq.push(*it);
			}
		}

		// At every few iterations we try to calculate the Dubins Path and add the states 
		// to the openlist. Once it collides we stop further iterations. 
		if( count%4==3 )
		{	
			
			vector<State> Path = heuristic.DubinShot(current,end,car.min_radius);

			State prev = current, check=current;
			for(vector<State>::iterator it= Path.begin(); it!=Path.end();it++)
			{
				State nextS = *it;
				
				if( !map.isValid(nextS))
					continue;

				if(map.isReached(nextS))
				{
					State temp=check;
					while( temp.parent != NULL )
					{
						path.push_back(temp);
						temp= *(temp.parent);
					}
					reverse(path.begin(), path.end());

			        if(DISPLAY_PATH)
					{
				     	display.clear();

			            display.draw_obstacles(obstacles, map_grid_resolution);
			            for(int i=0;i<=path.size();i++)
			            {
			                display.draw_car(path[i], car);
			                display.show(5);
			            } 
				     	display.show(1000);
				     	display.clear();
			        }					
					return path;
				
				}

				// This is to insure that consecutive points are not very close .Because of being very close 
				// consecutive points were assigned same parents and caused problems while storing path.
				if( sqrt(pow(nextS.x-prev.x,2) + pow(nextS.y-prev.y,2)) < 2 )
					continue;

				if( !map.checkCollision(nextS)&&!map.check_min_obs_dis(nextS,obs_dist)) //changed to start dubins shot only if the expected path is away from obstacles 
				{
					int prev_grid_x = roundDown(prev.x/planner_grid_resolution);;
					int prev_grid_y = roundDown(prev.y/planner_grid_resolution);
					int prev_grid_theta = ((int)(prev.theta*planner_grid_theta/(2*PI)))%planner_grid_theta;
					
					it->parent = &(visited_state[prev_grid_x][prev_grid_y][prev_grid_theta]);
					it->g = prev.g+1;
					it->h = heuristic.get_heuristic(*it,voronoi_values);

					int next_grid_x = roundDown(nextS.x/planner_grid_resolution);;
					int next_grid_y = roundDown(nextS.y/planner_grid_resolution);
					int next_grid_theta = ((int)(nextS.theta*planner_grid_theta/(2*PI)))%planner_grid_theta;
					
					visited_state[next_grid_x][next_grid_y][next_grid_theta] = *it;

					check=*it;
					prev=nextS;
					pq.push(*it);
				}
				else
				{
					Path.erase(it, Path.end());
					break;
				}
			}
			if(DISPLAY_SEARCH_TREE)
			{
				display.draw_dubins(Path);
				display.show(5);				
			}
		}
		count++;
	}
	cout<<"Goal cannot be reached"<<endl;
}



