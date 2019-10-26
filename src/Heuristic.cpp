#include "../include/Heuristic.hpp"
#include <boost/heap/fibonacci_heap.hpp>

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

struct point2d
{
    int x;
    int y;
    float cost;
    bool operator<(point2d other) const
    {
        return cost > other.cost;
    }
};

float distance (point2d source, point2d neighbor)
{
	return (sqrt((source.x-neighbor.x)*(source.x-neighbor.x)+(source.y-neighbor.y)*(source.y-neighbor.y)));
}

float resolution1 = 2.0;
Heuristic::Heuristic(Map map, float dijkstra_grid_resolution, State target, Vehicle vehicle)
{
    this->map = map;
    this->dijkstra_grid_resolution = dijkstra_grid_resolution;
    this->target = target;
    this->vehicle = vehicle;
    resolution1 = map.map_grid_resolution;
    dijkstra_grid_x = toInt(map.map_x/dijkstra_grid_resolution);
    dijkstra_grid_y = toInt(map.map_y/dijkstra_grid_resolution);

    int** obs = new int*[dijkstra_grid_x];
	for(int i=0;i<dijkstra_grid_x;i++)
	{
		obs[i]=new int[dijkstra_grid_y];
		for(int j=0;j<dijkstra_grid_y;j++)
			obs[i][j] = 0;
	}

    for(int i=0;i<map.map_grid_x;i++)
        for(int j=0;j<map.map_grid_y;j++)
            if(map.obs[i][j] > 0)
                obs[roundDown(i*map.map_grid_resolution/dijkstra_grid_resolution)][roundDown(j*map.map_grid_resolution/dijkstra_grid_resolution)] = 1;
    
    d = new float*[dijkstra_grid_x];
	for(int i=0;i<dijkstra_grid_x;i++)
	{
		d[i]=new float[dijkstra_grid_y];
		for(int j=0;j<dijkstra_grid_y;j++)
			d[i][j] = FLT_MAX;
	}

    bool** visited = new bool*[dijkstra_grid_x];
	for(int i=0;i<dijkstra_grid_x;i++)
	{
		visited[i]=new bool[dijkstra_grid_y];
		for(int j=0;j<dijkstra_grid_y;j++)
			visited[i][j] = false;
	}

    priority_queue <point2d, vector<point2d>> pq;

    point2d start, current, next;
    start.x = target.x/dijkstra_grid_resolution;
    start.y = target.y/dijkstra_grid_resolution;
    start.cost = 0;

    pq.push(start);

    while(!pq.empty())
    {
        current = pq.top();
        pq.pop();

        if(visited[current.x][current.y])
            continue;

        visited[current.x][current.y] = true;
        d[current.x][current.y] = current.cost;

        for(int i=-1;i<=1;i++)
            for(int j=-1;j<=1;j++)
            {
                if(current.x+i<0 || current.x+i>=dijkstra_grid_x || current.y+j<0 || current.y+j>=dijkstra_grid_x)
                    continue;
                
                if(obs[current.x+i][current.y+j] || visited[current.x+i][current.y+j])
                    continue;

                next.x = current.x + i;
                next.y = current.y + j;
                next.cost = current.cost + distance(current, next);

                if(next.cost<d[next.x][next.y])
                {
                    d[next.x][next.y] = next.cost;
                    pq.push(next);
                }
            }
    }
    
    return;

}

// ReedShepp's Path
double Heuristic::ReedSheppCost(State begin, State end, double radius)
{
    bool DEBUG=false;
    vector<State> nextStates;

    // Declaration of an OMPL Coordinate System  
    ob::StateSpacePtr space(new ompl::base::SE2StateSpace());

    // Declaration of two states in this Coordinate System  
    ob::State *start = space->allocState();
    ob::State *goal  = space->allocState();

    // Declaration of variables that configures it for 2D motion
    auto *s = start->as<ob::SE2StateSpace::StateType>();
    auto *t = goal->as<ob::SE2StateSpace::StateType>();
    
    // Reference :
    // http://docs.ros.org/diamondback/api/ompl/html/classompl_1_1base_1_1SE2StateSpace_1_1StateType.html
    
    s->setX(begin.x);
    s->setY(begin.y);
    s->setYaw(begin.theta);

    t->setX(end.x);
    t->setY(end.y);
    t->setYaw(end.theta);
    
    if(DEBUG)
    {
        double x1=s->getX(), y1=s->getY() ,theta1=s->getYaw();
        double x2=t->getX(), y2=t->getY() ,theta2=t->getYaw();
        cout<<x1<<" "<<y1<<" "<<theta1<<" "<<x2<<" "<<y2<<" "<<theta2<<endl;
    }
    
    // https://ompl.kavrakilab.org/classompl_1_1base_1_1ReedsSheppStateSpace.html
    ob::ReedsSheppStateSpace RSP(radius);
    auto Path=RSP.reedsShepp(start,goal);
    if(DEBUG)
        cout<<"Path Length : "<<Path.length()<<endl;
    return Path.length()*radius;
}


double Heuristic::get_heuristic(State pos,Mat final)
{
    //cout<<roundDown(pos.x/dijkstra_grid_resolution)<<","<<roundDown(pos.y/dijkstra_grid_resolution)<<endl;
    //cout<<dijkstra_grid_x<<","<<dijkstra_grid_y<<endl;
    float h1 = /*30.0-30.0*final.at<uchar>((int)pos.x/0.5,(int)pos.y/0.5)/255+*/dijkstra_grid_resolution * d[roundDown(pos.x/dijkstra_grid_resolution)][roundDown(pos.y/dijkstra_grid_resolution)];
    float h2 = ReedSheppCost(pos, target, vehicle.min_radius);
    /*cout << "Size : " << h1 << " " << h2 <<endl;
    cout << "Current position" << final.rows << " " << final.cols <<endl;
    */
    /*cout << " Heuristic ..................................................... " << endl;
   // cout << max(h1,h2) + 10.0-(10.0*final.at<uchar>((int)pos.x*2,(int)pos.y*2))/255 <<endl;
    
   */ 
  // cout<<"hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh" << h1<<"  "<<h2<<endl;
    return (max(h1, h2));//+12.0-12.0*final.at<uchar>((int)pos.x/0.5,(int)pos.y/0.5)/255;
}

vector<State> Heuristic::ReedSheppShot(State begin, State end, double radius)
{
    bool DEBUG=false;

    vector<State> nextStates;

    // Declaration of an OMPL Coordinate System  
    ob::StateSpacePtr space(new ompl::base::SE2StateSpace());

    // Declaration of two states in this Coordinate System  
    ob::State *start = space->allocState();
    ob::State *goal  = space->allocState();

    // Declaration of variables that configures it for 2D motion
    auto *s = start->as<ob::SE2StateSpace::StateType>();
    auto *t = goal->as<ob::SE2StateSpace::StateType>();
    
    // Reference :
    // http://docs.ros.org/diamondback/api/ompl/html/classompl_1_1base_1_1SE2StateSpace_1_1StateType.html
    
    s->setX(begin.x);
    s->setY(begin.y);
    s->setYaw(begin.theta);

    t->setX(end.x);
    t->setY(end.y);
    t->setYaw(end.theta);
    
    DEBUG = true;
    if(DEBUG)
    {
        double x1=s->getX(), y1=s->getY() ,theta1=s->getYaw();
        double x2=t->getX(), y2=t->getY() ,theta2=t->getYaw();
        cout<<"Inside ReedShepp "<<x1<<" "<<y1<<" "<<theta1<<" "<<x2<<" "<<y2<<" "<<theta2<<endl;
    }
    
    // https://ompl.kavrakilab.org/classompl_1_1base_1_1ReedsSheppStateSpace.html
    ob::ReedsSheppStateSpace RSP(radius);
    auto Path=RSP.reedsShepp(start,goal);


    if(DEBUG)
    {
        // This gives the total distance travelled along a curved path :ReedSheppCost
        cout<<"ReedShepp Distance : "<<Path.length()*radius<<endl;

        /*
        The type of ReedShepp curves possible are:
        {RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP},         
        {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP},        
        {RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP},       
        {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP},       
        {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP},    
        {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   
        {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},    
        {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},   
        {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   
        {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP},    
        {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},   
        {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},    
        {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},     
        {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},     
        {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},      
        {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},    
        {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT},  
        {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT}   

        Here in OMPL implementation the following enum are used :
        RS_NOP=0 , RS_LEFT=1, RS_STRAIGHT=2, RS_RIGHT=3
        */
        cout<<"RS Type : "<<Path.type_[0]<<" "<<Path.type_[1]<<" "<<Path.type_[2]<<" "<<Path.type_[3]<<" "<<Path.length_[4]<<endl;

        // It gives path length corresponding to each of the three component curves :
        cout<<"Length_0 :"<<Path.length_[0]*radius<<endl;
        cout<<"Length_1 :"<<Path.length_[1]*radius<<endl;
        cout<<"Length_2 :"<<Path.length_[2]*radius<<endl;
        cout<<"Length_3 :"<<Path.length_[3]*radius<<endl;
        cout<<"Length_4 :"<<Path.length_[4]*radius<<endl;
    }


    float stride = 1,d=2;
    State E,next;

    E.x = begin.x;
    E.y = begin.y;
    E.theta = begin.theta;

    for(int i=0;i<5;i++)
    {
        stride=1;

        if(Path.type_[i]==1)
        {
            // Left Turn
            if( abs(Path.length_[i])>0.1 )
            {
                if(Path.length_[i]>0)
                {
                    if(DEBUG)
                        cout<<"taking left"<<endl;

                    while(stride < abs(Path.length_[i]*radius))
                    {
                        next.x = (E.x - ( radius*sin(E.theta) - radius*sin(stride/radius + E.theta )));
                        next.y = (E.y + ( radius*cos(E.theta) - radius*cos( stride/radius + E.theta )));
                        next.theta = E.theta + (float)(stride/radius);
                        nextStates.push_back(next);        
                        
                        if(DEBUG)            
                            cout<<next.x<<" "<< next.y <<" "<<next.theta<<endl;
                        
                        stride+=d;
                    }

                    E.x = (E.x - (radius*sin(E.theta) - radius*sin( Path.length_[i] +E.theta )));
                    E.y = (E.y + (radius*cos(E.theta) - radius*cos( Path.length_[i] +E.theta )));
                    E.theta = E.theta + Path.length_[i] ;
                    if(DEBUG)
                        cout<<E.x<<" "<< E.y <<" "<<E.theta<<endl;

                    nextStates.push_back(E); 
                }
                else
                {
                    if(DEBUG)
                        cout<<"taking reverse left"<<endl;

                    stride = -1;
                    while(abs(stride) < abs(Path.length_[i]*radius))
                    {
                        next.x = (E.x - ( radius*sin(E.theta) - radius*sin(stride/radius + E.theta )));
                        next.y = (E.y + ( radius*cos(E.theta) - radius*cos( stride/radius + E.theta )));
                        next.theta = E.theta + (float)(stride/radius);
                        nextStates.push_back(next);        
                        
                        if(DEBUG)            
                            cout<<next.x<<" "<< next.y <<" "<<next.theta<<endl;
                        
                        stride-=d;
                    }

                    E.x = (E.x - (radius*sin(E.theta) - radius*sin( Path.length_[i] +E.theta )));
                    E.y = (E.y + (radius*cos(E.theta) - radius*cos( Path.length_[i] +E.theta )));
                    E.theta = E.theta + Path.length_[i] ;
                    if(DEBUG)
                        cout<<E.x<<" "<< E.y <<" "<<E.theta<<endl;

                    nextStates.push_back(E); 
                }       
            }
        }
        else if(Path.type_[i]==3)
        {
            // Right Turn
            radius = (float)(-1*radius);
            if( abs(Path.length_[i])>0.1 )
            {
                if(Path.length_[i]>0)
                {
                    if(DEBUG)
                        cout<<"taking right"<<endl;

                    while(stride < abs(Path.length_[i]*radius))
                    {
                        next.x = (E.x - ( radius*sin(E.theta) - radius*sin(stride/radius + E.theta )));
                        next.y = (E.y + ( radius*cos(E.theta) - radius*cos(stride/radius + E.theta )));
                        next.theta = E.theta + (float)(stride/radius);
                        nextStates.push_back(next);        

                        if(DEBUG)            
                            cout<<next.x<<" "<< next.y <<" "<<next.theta<<endl;
                        
                        stride+=d;
                    }

                    E.x = (E.x - (radius*sin(E.theta) - radius*sin( -Path.length_[i] +E.theta )));
                    E.y = (E.y + (radius*cos(E.theta) - radius*cos( -Path.length_[i] +E.theta )));
                    E.theta = E.theta - Path.length_[i] ;
                    if(DEBUG)
                        cout<<E.x<<" "<< E.y <<" "<<E.theta<<endl;

                    nextStates.push_back(E);  
                }
                else
                {
                    if(DEBUG)
                        cout<<"taking reverse right"<<endl;

                    stride=-1;
                    while(abs(stride) < abs(Path.length_[i]*radius))
                    {
                        next.x = (E.x - ( radius*sin(E.theta) - radius*sin(stride/radius + E.theta )));
                        next.y = (E.y + ( radius*cos(E.theta) - radius*cos(stride/radius + E.theta )));
                        next.theta = E.theta + (float)(stride/radius);
                        nextStates.push_back(next);        

                        if(DEBUG)            
                            cout<<next.x<<" "<< next.y <<" "<<next.theta<<endl;
                        
                        stride-=d;
                    }

                    E.x = (E.x - (radius*sin(E.theta) - radius*sin( -Path.length_[i] +E.theta )));
                    E.y = (E.y + (radius*cos(E.theta) - radius*cos( -Path.length_[i] +E.theta )));
                    E.theta = E.theta - Path.length_[i] ;
                    if(DEBUG)
                        cout<<E.x<<" "<< E.y <<" "<<E.theta<<endl;

                    nextStates.push_back(E);   
                }      
            }
            radius = (float)(-1*radius);
        } 
        else if(Path.type_[i]==2)
        {
            // Straight
            if(abs(Path.length_[i])>0.1)
            {
                if(Path.length_[i]>0)
                {
                    if(DEBUG)
                        cout<<"taking Straight"<<endl;

                    while(stride < Path.length_[i]*radius)
                    {
                        next.x = E.x + stride*cos(E.theta);
                        next.y = E.y + stride*sin(E.theta);
                        next.theta = E.theta;
                        nextStates.push_back(next);

                        if(DEBUG)            
                            cout<<next.x<<" "<< next.y <<" "<<next.theta<<endl;

                        stride+=d;
                    }

                    E.x = E.x + Path.length_[i]*radius*cos(E.theta);
                    E.y = E.y + Path.length_[i]*radius*sin(E.theta);

                    if(DEBUG)
                        cout<<E.x<<" "<< E.y <<" "<<E.theta<<endl;

                    nextStates.push_back(E);
                }
                else
                {
                    if(DEBUG)
                        cout<<"taking reverse"<<endl;

                    stride = -1;
                    while(abs(stride) < abs(Path.length_[i]*radius))
                    {
                        next.x = E.x + stride*cos(E.theta);
                        next.y = E.y + stride*sin(E.theta);
                        next.theta = E.theta;
                        nextStates.push_back(next);

                        if(DEBUG)            
                            cout<<next.x<<" "<< next.y <<" "<<next.theta<<endl;

                        stride-=d;
                    }

                    E.x = E.x + Path.length_[i]*radius*cos(E.theta);
                    E.y = E.y + Path.length_[i]*radius*sin(E.theta);

                    if(DEBUG)
                        cout<<E.x<<" "<< E.y <<" "<<E.theta<<endl;

                    nextStates.push_back(E);
                }

                
            }
        }
        else
        {
            continue;
        }

    }
 
    
    if(DEBUG)
        for(int i=0;i<nextStates.size();i++)
        {
            cout<<"x: "<<nextStates[i].x<<" y: "<<nextStates[i].y<<endl;
        }

    if(DEBUG)
        cout<<"End of ReddShepp Shot"<<endl;

    DEBUG = false;
    return nextStates;
}
