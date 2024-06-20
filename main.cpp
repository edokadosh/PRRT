#include <ctime>
#include "rrt.h"
#include "draw.h"
#include "inputParser.h"
#include "cudaRRTSolver.h"

using namespace std;

void _measureTime(void(*f)(Solver&, std::string), Solver &s, std::string resPath) {
    clock_t begin = clock();
    (f)(s, resPath);
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    cout << "Time elapsed: "<< elapsed_secs << " seconds" << endl;
}

void useSolver(Solver &s, std::string resPath) {
    s.solve();
    s.draw(resPath);
}

void useSolverNoDraw(Solver &s, std::string resPath = "") {
    s.solve();
}

int main(int argc, char** argv) {
    InputParser input(argc, argv);

    const std::string &path = input.getCmdOption("-scene");
    if (path.empty()) {
        cout << "Error: invalid arguments. usage: rrt -scene scene.json]" << endl;
        return 0;
    }

    const std::string &seedString = input.getCmdOption("-seed");
    int seed = 0;
    if (seedString.empty()) {
        seed = std::time(0);
        cout << "Seed set to random: " << seed << endl;
    }
    else {
        seed = stoi(seedString);
        cout << "Seed set to: " << seed << endl;
    }
    std::srand(seed);

    Scene s = Scene(string(path));

    void(*funcToRun)(Solver&, std::string) = useSolver;

    if(input.cmdOptionExists("-q")){ //quiet
        funcToRun = useSolverNoDraw;
        cout << "Will not draw result when done!" << endl;
    }

    const std::string &solverString = input.getCmdOption("-solver");

    if(solverString.empty() || solverString == "regular"){
        cout << "Solving with regular" << endl;

        RRT solver(s, 100, 100);
        _measureTime(funcToRun, solver, "regular_res.bmp");
    }

    if (solverString == "cuda"){
        const std::string &numThreadsStr = input.getCmdOption("-num_threads");
        const std::string &blockSizeStr = input.getCmdOption("-block_size");
        int numThreads, blockSize;

        if (numThreadsStr.empty()) {
            numThreads = 1 << 2;
        } else {
            numThreads = stoi(numThreadsStr);
        }
        if (blockSizeStr.empty()) {
            blockSize = 1 << 1;
        } else {
            blockSize = stoi(blockSizeStr);
        }

        cout << "Solving with cuda" << endl;
        cout << "using " << numThreads << " threads with block size of " << blockSize << endl;

        CudaRRTSolver solver(s, 100, 100, seed, numThreads, blockSize);
        _measureTime(funcToRun, solver, "cuda_res.bmp");
        // cout << solver.checkDestinationReached() << endl;
    }


    cout << "done!!" << endl;

    return 0;
}