#pragma once
#include "solver.h"


class RRT : public Solver{
private:
    int nodeCnt = 0, goalIndex = -1 ; 
    bool destinationReached;

    void RRTiteration();
    bool getDestinationReached();

public: 
    RRT(Scene scene, int xBuckets, int yBuckets) 
    : Solver(scene, xBuckets, yBuckets) {
        nodes.push_back(s.start); 
        parent.push_back(0); 
        cost.push_back(0);
    }
    void solve() override;
};