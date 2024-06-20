#include "cudaRRT.cuh"

//gets res Tree with one starting point, and develops the tree from it using rrt
__global__ 
void cudaRRT(CudaScene s, CudaTree *res) {
    int idx = threadIdx.x+blockDim.x*blockIdx.x;
    CudaTree *myRes = &res[idx];
    // printf("Kernel %d started\n", idx);

    long long int start_time = clock64(); 

    CudaPoint newPoint, nearestPoint, freePoint, step; 
	int nearestIndex = 0; 

    while (myRes->nodeCnt < NUM_NODES_TO_ADD and clock64() < start_time+MAX_KERNEL_CYCLES) {
        do {
            newPoint = s.randomPoint(); 

            // Find nearest point to the newPoint such that the next node 
            // be added in graph in the (nearestPoint, newPoint) while being obstacle free
            nearestPoint = myRes->nodes[0].p; 
            nearestIndex = 0;
            for(int i = 0; i < myRes->nodeCnt; i++) {
                CudaPoint pnt = myRes->nodes[i].p; 

                if((pnt.distance(newPoint) - nearestPoint.distance(newPoint)) <= _EPS)
                    nearestPoint = pnt, nearestIndex = i ; 
            }
            freePoint = nearestPoint;
            step = (newPoint-nearestPoint).normalize() * STEP_SIZE;


            while(freePoint.distance(newPoint) > STEP_SIZE and s.isInBounds(freePoint+step) and s.isEdgeObstacleFree(nearestPoint, freePoint+step)) {
                freePoint+=step;
		    }
        } while ((nearestPoint == freePoint) or (!s.isEdgeObstacleFree(nearestPoint, freePoint)));

		myRes->addNode(freePoint, nearestIndex, myRes->nodes[nearestIndex].cost);
    }
    
    // long long int stop_time = clock64(); 
    // printf("Kernel %d done in %ld cycles\n", idx, (stop_time - start_time));
}

__global__ void initRandom(curandStateMRG32k3a *state, int seed){
    int idx = threadIdx.x+blockDim.x*blockIdx.x;
    curand_init(seed, idx, 0, &state[idx]);
}

void runCudaRRT(CudaScene s, CudaTree *res, int numThreads, int blockSize) {
    int numBlocks = numThreads/blockSize;

    cudaRRT<<<numBlocks, blockSize>>>(s, res);
    cudaDeviceSynchronize();
}

void runInitRandom(curandStateMRG32k3a *state, int seed, int numThreads, int blockSize) {
    int numBlocks = numThreads/blockSize;

    initRandom<<<numBlocks, blockSize>>>(state, seed);
    cudaDeviceSynchronize();
}

__host__ __device__
inline bool CudaScene::isInBounds(CudaPoint a) {
    return xMin < a.x && a.x < xMax && yMin < a.y && a.y < yMax;
}

__host__ __device__
inline int CudaScene::xToi(cuft  x) {
    return (x - xMin)/xJump;
}

__host__ __device__
inline int CudaScene::yToj(cuft  y) {
    return (y - yMin)/yJump;
}

__device__
CudaPoint CudaScene::randomPoint() {
    int idx = threadIdx.x+blockDim.x*blockIdx.x;
    return CudaPoint(curand_uniform_double(&randState[idx])*(xMax-xMin)+xMin, curand_uniform_double(&randState[idx])*(yMax-yMin)+yMin);
}

__host__ __device__
bool CudaScene::isEdgeObstacleFree(CudaPoint a, CudaPoint b) {

    if (!(isInBounds(a) && isInBounds(b))) {
        printf("(%f,%f), (%f,%f) \n", a.x, a.y, b.x, b.y);
        assert(0);
        return false;
    }

    if (a.x > b.x) {
        CudaPoint temp = a;
        a = b;
        b = temp;
    } // a.x < b.x

    for (int i = xToi(a.x); i <= xToi(b.x); i++) {
        cuft yStart = yOfXOnLine(a,b,max(i*xJump + xMin, a.x));
        cuft yEnd = yOfXOnLine(a,b,min((i+1)*xJump + xMin, b.x));
        if (yStart > yEnd) {
            cuft temp = yStart;
            yStart = yEnd;
            yEnd = temp;
        } // yStart < yEnd

        for (int j = yToj(yStart); j <= yToj(yEnd) ; j++) {
            for (int polyIndexInBucket = 0 ; polyIndexInBucket < collisionDetection[i][j].obsCnt ; polyIndexInBucket++) {
                if(lineSegmentIntersectsPolygon(a, b, obstacles[collisionDetection[i][j].obstacleIndexes[polyIndexInBucket]])) {
                    return false; 
                }
            }
        }
    }

    return true; 
}