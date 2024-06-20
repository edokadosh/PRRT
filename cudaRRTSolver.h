#pragma once

#include <iostream>
#include <vector>
#include <utility>
#include <cuda.h>
#include <flann/flann.hpp>
// #include "CXXGraph/CXXGraph.hpp"
#include "solver.h"
#include "cudaRRT.cuh" 
#include <stdio.h>
#include "draw.h"

using namespace flann;
// using namespace CXXGraph;

#define DEG 2 // wont work to just change
#define NUM_NODES_TO_COMBINE 10
#define NEAREST_NEIGHBORS 10

class CudaRRTSolver : public Solver {
    int numThreads;
    int blockSize;
    
    CudaTree *resultTrees;
    CudaScene cuScene;

    // for deallocation:
    CudaCollisionBucket *hostBuckets; // arrray of buckets, such that their obstacleIndexes are pointing to the device, for memory deallocation
    CudaCollisionBucket *cudaBuckets; // pointer to the array of bucket on the devide, for memory deallocation

    vector<pair<int,int>> edges; 

    void sceneToCuda(Scene scene, int xBuckets, int yBuckets);

public:
    CudaRRTSolver() {
        throw std::invalid_argument("Error: invalid creation of CudaRRTSolver");
    }
    CudaRRTSolver(Scene scene, int xBuckets, int yBuckets, int cudaSeed, int numThreads, int blockSize);

    void solve() override;

    void combineTrees();

    void draw(string path) override;

    bool checkDestinationReached();

    ~CudaRRTSolver();
};
