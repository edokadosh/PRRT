#include "cudaRRTSolver.h"

CudaRRTSolver::CudaRRTSolver(Scene scene, int xBuckets, int yBuckets, int cudaSeed, int _numThreads, int _blockSize) : Solver(scene, xBuckets, yBuckets), numThreads(_numThreads), blockSize(_blockSize) { 
    cudaMalloc((void **)&cuScene.randState,numThreads*sizeof(curandStateMRG32k3a));

    sceneToCuda(s, xBuckets, yBuckets);

    runInitRandom(cuScene.randState, cudaSeed, numThreads, blockSize);

    cudaMalloc((void **)&resultTrees, numThreads*sizeof(CudaTree));
}

void CudaRRTSolver::sceneToCuda(Scene scene, int xBuckets, int yBuckets) {
    cuScene.xMin = scene.xMin;
    cuScene.xMax = scene.xMax;
    cuScene.yMin = scene.yMin;
    cuScene.yMax = scene.yMax;

    cuScene.xJump = scene.xJump;
    cuScene.yJump = scene.yJump;

    cuScene.start.x = scene.start.x;
    cuScene.start.y = scene.start.y;
    cuScene.end.x = scene.end.x;
    cuScene.end.y = scene.end.y;


    // copy obstacles to device memory
    int numObstacles = scene.obstacles.size();

    CudaPolygon *hostObstacles = (CudaPolygon*)malloc(numObstacles*sizeof(CudaPolygon));
    int obsIdx = 0;

    for (auto obs : scene.obstacles) {
        hostObstacles[obsIdx] = CudaPolygon();
        for (int i = 0 ; i < 4 && i < obs.points.size(); i++) {
            hostObstacles[obsIdx].addPoint(CudaPoint(obs.points[i].x, obs.points[i].y));
        }
        obsIdx++;
    }

    cudaMalloc((void **)&cuScene.obstacles,numObstacles*sizeof(CudaPolygon));
    cudaMemcpy(cuScene.obstacles,hostObstacles,numObstacles*sizeof(CudaPolygon),cudaMemcpyHostToDevice);
    
    free(hostObstacles);


    // create collisionDetection bucket hash table in device memory 

    vector<vector<vector<int>>> hostCollisionIndexes; // creating collison detection polygon indexes data struct

    for (int i = 0 ; i < xBuckets ; i++) {
        hostCollisionIndexes.push_back(vector<vector<int>>());
        for (int j = 0 ; j < xBuckets ; j++) {
            hostCollisionIndexes[i].push_back(vector<int>());
        }
    }

    for (int o = 0 ; o < scene.obstacles.size() ; o++) {
        auto obs = scene.obstacles[o];
        cuft polyXMin = obs.points[0].x;
        cuft polyXMax = obs.points[0].x;
        cuft polyYMin = obs.points[0].y;
        cuft polyYMax = obs.points[0].y;

        for (Point p : obs.points) {
            polyXMin = polyXMin < p.x ? polyXMin : p.x;
            polyXMax = polyXMax > p.x ? polyXMax : p.x;
            polyYMin = polyYMin < p.y ? polyYMin : p.y;
            polyYMax = polyYMax > p.y ? polyYMax : p.y;
        }

        for (int i = int((polyXMin - scene.xMin) / scene.xJump) ; scene.xMin+(i*scene.xJump) < polyXMax ; i++) {
            for (int j = int((polyYMin - scene.yMin) / scene.yJump) ; scene.yMin+(j*scene.yJump) < polyYMax ; j++) {
                hostCollisionIndexes[i][j].push_back(o);
            }
        }
    }



    // create the bucket on the host, but for each bucket create the index list on the device
    hostBuckets = (CudaCollisionBucket*)malloc(xBuckets*yBuckets*sizeof(CudaCollisionBucket));

    for (int i = 0 ; i < xBuckets ; i++) {
        for (int j = 0 ; j < yBuckets ; j++) {
            int *newBucketIndexes = (int*)malloc(hostCollisionIndexes[i][j].size()*sizeof(int));
            for (int k = 0 ; k < hostCollisionIndexes[i][j].size() ; k++) {
                newBucketIndexes[k] = hostCollisionIndexes[i][j][k];
            }
            cudaMalloc((void **)&hostBuckets[i*yBuckets+j].obstacleIndexes,hostCollisionIndexes[i][j].size()*sizeof(int));
            cudaMemcpy(hostBuckets[i*yBuckets+j].obstacleIndexes,newBucketIndexes,hostCollisionIndexes[i][j].size()*sizeof(int),cudaMemcpyHostToDevice);
            free(newBucketIndexes);
            hostBuckets[i*yBuckets+j].obsCnt = hostCollisionIndexes[i][j].size();
        }
    }

    // copy the buckets to the device
    cudaMalloc((void **)&cudaBuckets,xBuckets*yBuckets*sizeof(CudaCollisionBucket));
    cudaMemcpy(cudaBuckets,hostBuckets,xBuckets*yBuckets*sizeof(CudaCollisionBucket),cudaMemcpyHostToDevice);

    // create the array of pointers to the buckets,  
    // this array has xBuckets items, each being a pointer to the buckets themselves on the device

    CudaCollisionBucket **hostCollisionDetection = (CudaCollisionBucket**)malloc(sizeof(CudaCollisionBucket*)*xBuckets);

    for (int i = 0 ; i < xBuckets ; i++) {
        hostCollisionDetection[i] = cudaBuckets+(i*yBuckets);
    }

    // copy the array of pointers to buckets to the device

    cudaMalloc((void **)&cuScene.collisionDetection, sizeof(CudaCollisionBucket*)*xBuckets);
    cudaMemcpy(cuScene.collisionDetection,hostCollisionDetection, sizeof(CudaCollisionBucket*)*xBuckets, cudaMemcpyHostToDevice);
    free(hostCollisionDetection);
}


void CudaRRTSolver::solve() {
    CudaTree newTree;
    Point randomPoint;
    newTree.nodeCnt = 1;
    CudaNode startNode(CudaPoint(s.start.x, s.start.y), 0, 0);


    for (int i = 0 ; i < numThreads ; i++) {
        // printPoint(startNode.p);
        // cout << endl;

        newTree.nodes[0] = startNode;
        
        cudaMemcpy(&resultTrees[i], &newTree, sizeof(CudaTree), cudaMemcpyHostToDevice);

        randomPoint = s.randomFreePoint();
        startNode.p = CudaPoint(randomPoint.x, randomPoint.y);
    }

    cout << "Starting to solve on cuda..." << endl;
    clock_t begin = clock();

    runCudaRRT(cuScene, resultTrees, numThreads, blockSize);

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "Solving on cuda took " << elapsed_secs << " seconds" << endl;

    cout << "Starting to combine trees..." << endl;
    begin = clock();

    combineTrees();

    end = clock();
    elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "Combining trees took " << elapsed_secs << " seconds" << endl;

    cout << "Solved!" << endl;

    cout << nodes.size() << " nodes" << endl;
    cout << edges.size() << " edges" << endl;
}

void CudaRRTSolver::combineTrees() {
    CudaTree hostResultTrees[numThreads];

    cudaMemcpy(&hostResultTrees,resultTrees,numThreads*sizeof(CudaTree),cudaMemcpyDeviceToHost);

    int sumNodeCnt = 0;
    vector<int> queryPoints;

    // start by adding all the clusters
    for (int t = 0 ; t < numThreads ; t++) {
        for (int n = 0 ; n < hostResultTrees[t].nodeCnt ; n++) {
            // cout << t << ", " << n << " : ";
            // printPoint(Point(hostResultTrees[t].nodes[n].p.x, hostResultTrees[t].nodes[n].p.y));
            // cout << endl;

            nodes.push_back(Point(hostResultTrees[t].nodes[n].p.x, hostResultTrees[t].nodes[n].p.y));
            edges.push_back(pair<int,int>(sumNodeCnt+n, sumNodeCnt + hostResultTrees[t].nodes[n].parent));
            if (n > hostResultTrees[t].nodeCnt - NUM_NODES_TO_COMBINE) {
                queryPoints.push_back(nodes.size()-1); // add index of last added point
            }
        }
        sumNodeCnt += hostResultTrees[t].nodeCnt;
    }

    // now use knn to combine the clusters

    double *pointsData = (double*)malloc(sizeof(double)*DEG*sumNodeCnt);

    for (int p = 0 ; p < nodes.size() ; p++) {
        pointsData[DEG*p] = nodes[p].x;
        pointsData[DEG*p+1] = nodes[p].y;
    }

    Matrix<double> dataset(pointsData, sumNodeCnt, DEG);


    double *queryData = (double*)malloc(sizeof(double)*DEG*queryPoints.size());

    for (int p = 0 ; p < queryPoints.size() ; p++) {
        queryData[DEG*p] = nodes[queryPoints[p]].x;
        queryData[DEG*p+1] = nodes[queryPoints[p]].y;
    }

    Matrix<double> query(queryData, sumNodeCnt, DEG);

    Matrix<int> indices(new int[query.rows*NEAREST_NEIGHBORS], query.rows, NEAREST_NEIGHBORS);
    Matrix<double> dists(new double[query.rows*NEAREST_NEIGHBORS], query.rows, NEAREST_NEIGHBORS);

    Index<L2<double> > index(dataset, flann::KDTreeIndexParams(4));
    index.buildIndex();

    index.knnSearch(query, indices, dists, NEAREST_NEIGHBORS, flann::SearchParams(128));

    for (int q = 0 ; q < queryPoints.size() ; q++) {
        for (int i = 0 ; i < NEAREST_NEIGHBORS ; i ++) {
            if (!(nodes[queryPoints[q]] == nodes[indices[q][i]]) and s.isEdgeObstacleFree(nodes[queryPoints[q]], nodes[indices[q][i]]))
                edges.push_back(pair<int,int>(queryPoints[q], indices[q][i]));
        }
    }

    // TODO (but not really important right now): update edges
    // I think I just wont tupdate them because its unnecessary

    free(pointsData);
    free(queryData);
    delete[] indices.ptr();
    delete[] dists.ptr();
}

// bool CudaRRTSolver::checkDestinationReached() {
//     // chekc if some node is reachable from destination
//     bool isReachable = false
//     nodes.push_back(scene.end);
//     for (int i = 0 ; i < nodes.size() ; i++) {
//         if s.isEdgeObstacleFree(nodes[i], scene.end) {
//             isReachable = true;
//             edge.push_back(i, nodes.size())
//         }
//     }

//     if (!isReachable) {
//         return false;
//     }

//     Graph<int> g;
//     CXXGRAPH::Node<int> *graphNodes = new CXXGRAPH::Node<int>[nodes.size()];
//     for (int i = 0 ; i < nodes.size() ; i++) {
//         graphNodes[i] = CXXGRAPH::Node<int>(std::to_string(nodes[i]), nodes[i])
//     }
    
//     for (int i = 0 ; i < edges.size() ; i++) {
//         std::pair<const CXXGRAPH::Node<int> *, const CXXGRAPH::Node<int> *> pairNode(&graphNodes[edges[i].first], &graphNodes[edges[i].second]);
//         CXXGRAPH::UndirectedEdge<int> edge(i, pairNode);
//         g.addEdge(edge); 
//     }

//     auto res = g.depth_first_search(graphNodes[0], graphNodes[nodes.size()]);
//     cout << res << endl;

//     delete[] graphNodes;

//     return true;
// }

void CudaRRTSolver::draw(string path) {
	Draw d(IMAGE_W,IMAGE_H, s.xMin, s.xMax, s.yMin, s.yMax);

	for (auto obs : s.obstacles) {
    	d.drawPolygon(obs, RED);
	}

	d.drawPoint(s.start, GREEN);
	d.drawPoint(s.end, BLUE);

    for (int i = 0 ; i < edges.size() ; i++) {
        d.drawEdge(nodes[edges[i].first], nodes[edges[i].second], GREEN);
    }

    if (GRID) {
        for (int i = s.xMin + 1 ; i < s.xMax ; i++) {
            d.drawEdge(Point(i, s.yMin), Point(i, s.yMax), PURPLE);
        }

        for (int i = s.yMin + 1 ; i < s.yMax ; i++) {
            d.drawEdge(Point(s.xMin, i), Point(s.xMax, i), PURPLE);
        }
    }


    cout << "saving to: " << path << endl;

    d.save(path);
}

CudaRRTSolver::~CudaRRTSolver() {
    cudaFree(resultTrees);
    cudaFree(cuScene.obstacles);
    
    for (int i = 0 ; i < s.xBuckets ; i++) {
        for (int j = 0 ; j< s.yBuckets ; j++) {
            cudaFree(hostBuckets[i*s.yBuckets+j].obstacleIndexes);
        }
    }
    free(hostBuckets);
    cudaFree(cudaBuckets);

    cudaFree(cuScene.collisionDetection);
    cudaFree(cuScene.randState);
}