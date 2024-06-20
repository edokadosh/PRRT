#include <cuda.h>
#include <stdio.h>
#include <stdlib.h>

__global__ void matinit(int N, float* M);
__global__ void matzero(int N, float* M);
__global__ void matmul(int N, float* M1, float* M2, float *M3);

int main(int argc, char** argv)
{   
    if (argc < 2) {
        printf("Usage:  ./matmul N\n");
        printf("where N is a matrix dimension, ");
        printf("100 <= N <= 30000.\n");
        exit(0);
    }

    int N = atoi(argv[1]);

    if (N < 100 || N > 30000) {
        printf("Dimension is out of bounds!\n");
        exit(1);
    }

    // we represent each matrix as a 1-d array of N*N elements
    float* M1_d;
    float* M2_d;
    float* M3_d;

    // allocate memory on the device (i.e. the gpu) for matrices
    cudaMalloc( (void**) &M1_d, N*N*sizeof(float) );
    cudaMalloc( (void**) &M2_d, N*N*sizeof(float) );
    cudaMalloc( (void**) &M3_d, N*N*sizeof(float) );

    // compute number of 32-thread blocks required to
    // cover N iterations
    int numblocks = (int)( 0.5 + (N / 32.0) ); // ceil(N/32)

    // initialise matrices.  It is paramount to set M3 to zero,
    // the algorithm will not work correctly otherwise
    matinit<<<numblocks,32>>>(N, M1_d);
    matinit<<<numblocks,32>>>(N, M2_d);
    matzero<<<numblocks,32>>>(N, M3_d);
    cudaDeviceSynchronize();

    printf("Starting...");
    clock_t start = clock();
    matmul<<<numblocks,32>>>(N, M1_d, M2_d, M3_d);
    cudaDeviceSynchronize();
    clock_t stop = clock();
    printf("\nFinished!\n");
    double s = (double)(stop - start) / CLOCKS_PER_SEC;
    printf("Time taken: %0.3f seconds\n\n", s);

    // free memory
    cudaFree(M3_d);
    cudaFree(M2_d);
    cudaFree(M1_d);

    exit(0);
}

__global__ void matmul(int N, float* M1, float* M2, float *M3)
{
    int i = threadIdx.y*32 + threadIdx.x;

    if (i < N) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
                M3[i*N + j] += M1[i*N + k] * M2[k*N + j];
            }
        }
    }
}

__global__ void matinit(int N, float* M)
{
    int j = threadIdx.y*32 + threadIdx.x;

    if (j < N) {
        for (int i = 0; i < N; i++)
            M[i*N + j] = 100.0 - 200.0 * j / N;
    }
}

__global__ void matzero(int N, float* M)
{
    int j = threadIdx.y*32 + threadIdx.x;

    if (j < N) {
        for (int i = 0; i < N; i++)
            M[i*N + j] = 0.0;
    }
}