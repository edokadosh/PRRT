#pragma once

#ifndef SOLVER_H
#define SOLVER_H

#include "scene.h"
#include "draw.h"


#define STEP_SIZE 0.2

class Solver {
protected:
    vector < Point > nodes ; 
    vector < int > parent; 
    vector < double > cost; 
    Scene s;
public:
    Solver() {
        throw std::invalid_argument("Error: invalid creation of Solver");
    }
    Solver(Scene scene, int xBuckets, int yBuckets);
    virtual void solve() = 0;
    virtual void draw(string path);
};

#endif