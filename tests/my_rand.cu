#include <stdio.h>
#include <stdlib.h>

#include <cuda_runtime.h>
#include <curand_kernel.h>

#define CUDA_CALL(x) do { if((x) != cudaSuccess) { \
    printf("Error at %s:%d\n",__FILE__,__LINE__); \
    return EXIT_FAILURE;}} while(0)

__global__ void generate_uniform_kernel(curandState *state,
                                int n,
                                unsigned int *result)
{
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int count = 0;
    float x;
    /* Copy state to local memory for efficiency */
    curandState localState = state[id];
    /* Generate pseudo-random uniforms */
    for(int i = 0; i < n; i++) {
        x = curand_uniform(&localState);
        /* Check if > .5 */
        if(x > .5) {
            count++;
        }
    }
    /* Copy state back to global memory */
    state[id] = localState;
    /* Store results */
    result[id] += count;
}

__global__ void setup_kernel(curandState *state)
{
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    /* Each thread gets same seed, a different sequence
       number, no offset */
    curand_init(1234, id, 0, &state[id]);
}

int main(int argc, char *argv[])
{
    const unsigned int threadsPerBlock = 64;
    const unsigned int blockCount = 64;
    const unsigned int totalThreads = threadsPerBlock * blockCount;

    unsigned int i;
    unsigned int total;
    curandState *devStates;
    unsigned int *devResults, *hostResults;
    int sampleCount = 10000;
    bool doubleSupported = 0;
    int device;
    struct cudaDeviceProp properties;

    /* check for double precision support */
    CUDA_CALL(cudaGetDevice(&device));
    CUDA_CALL(cudaGetDeviceProperties(&properties,device));
    if ( properties.major >= 2 || (properties.major == 1 && properties.minor >= 3) ) {
        doubleSupported = 1;
    }

    /* Allocate space for results on host */
    hostResults = (unsigned int *)calloc(totalThreads, sizeof(int));

    /* Allocate space for results on device */
    CUDA_CALL(cudaMalloc((void **)&devResults, totalThreads *
              sizeof(unsigned int)));

    /* Set results to 0 */
    CUDA_CALL(cudaMemset(devResults, 0, totalThreads *
              sizeof(unsigned int)));

    /* Allocate space for prng states on device */
    CUDA_CALL(cudaMalloc((void **)&devStates, totalThreads *sizeof(curandState)));

    /* Setup prng states */
    setup_kernel<<<64, 64>>>(devPHILOXStates);

    /* Generate and use uniform pseudo-random  */
    generate_uniform_kernel<<<64, 64>>>(devStates, sampleCount, devResults);

    /* Copy device memory to host */
    CUDA_CALL(cudaMemcpy(hostResults, devResults, totalThreads *
        sizeof(unsigned int), cudaMemcpyDeviceToHost));

    /* Show result */
    total = 0;
    for(i = 0; i < totalThreads; i++) {
        total += hostResults[i];
    }
    printf("Fraction of uniforms > 0.5 was %10.13f\n",
        (float)total / (totalThreads * sampleCount * 50.0f));

    /* Cleanup */
    CUDA_CALL(cudaFree(devStates));
    
    CUDA_CALL(cudaFree(devResults));
    free(hostResults);
    printf("^^^^ kernel_example PASSED\n");
    return EXIT_SUCCESS;
}