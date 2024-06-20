#pragma once
#include <iostream>
#include <cstdio>
#include <random>
#include <vector>
#include <ctime>
#include <fstream>
#include <json/json.h>
#include "geometry.h"

using namespace std; 

const double INF = 1e18;

class Solver;
class RRT;

class Scene {
public:
    vector<Polygon> obstacles; 
    vector<vector<vector<Polygon>>> collisionDetection;
    _ftype xMin, xMax, yMin, yMax;
    _ftype xJump, yJump;
    Point start, end;
    int xBuckets, yBuckets;

    // check if point is in bounds
    inline bool isInBounds(Point a) {
        return this->xMin < a.x && a.x < this->xMax && this->yMin < a.y && a.y < this->yMax;
    }
    inline int xToi(_ftype x) {
        return (x - this->xMin)/this->xJump;
    }
    inline int yToj(_ftype y) {
        return (y - this->yMin)/this->yJump;
    }

public:
    Scene() {};
    Scene(vector<Polygon> _obstacles, Point _start, Point _end) : obstacles(_obstacles), start(_start), end(_end) {}
    Scene(std::string path);

    Point randomPoint();
    Point randomFreePoint();

    // create the initCollisionDetection object as a hashmap of obstacles
    void initCollisionDetection(int xBuckets, int yBuckets);
    bool isEdgeObstacleFree(Point a, Point b);
    bool isPointFree(Point p);
   
    friend class Solver;
    friend class RRT;
    friend class CudaRRTSolver;
};

inline void printPoint(Point p) {
    cout << "(" << p.x << ", " << p.y << ") ";
}

inline void printPoly(Polygon poly) {
    for (auto p : poly.points) {
        printPoint(p);
    }
}