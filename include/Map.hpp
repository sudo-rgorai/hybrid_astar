#ifndef MAP_HPP
#define MAP_HPP

#include"../src/Vehicle.cpp"
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

typedef struct _border
{
    int Xmax;
    int Xmin;
    int Ymax;
    int Ymin;
}border;

class Map
{
    public:
        
        State end;
        vector<border> bPoints; 
        vector< vector<Point> > obs;

        Mat obs_img;
        int** acc_obs_map;
        vector< vector< bool > > obs_map;

        int RES;
        int VISX;
        int VISY;
        int MAP_THETA;

        Vehicle car;

        Map()
        {
            
        }

        Map(pair< vector<vector<bool> >, double > , State , int, int);

        bool isReached(State curr);
        bool isValid(Point);
        bool isValid(State);

        void initCollisionCheckerSat();
        bool checkCollisionSat(State pos);
        bool helperSAT(vector <Point> v1,vector <Point> v2);

        void initCollisionChecker();
        bool checkCollision(State pos);

};
#endif
