import time
from subprocess import Popen, PIPE
from tqdm import tqdm
import pickle
import matplotlib.pylab as plt

def wait_timeout(proc, seconds):
    """Wait for a process to finish, or raise exception after timeout"""
    start = time.time()
    end = start + seconds
    interval = min(seconds / 1000.0, .25)

    while True:
        result = proc.poll()
        if result is not None:
            return result
        if time.time() >= end:
            return None
        time.sleep(interval)


batch = 6

def get_res(s, before, after):
    idx = s.find(before) + len(before)+1
    return s[idx:s.find(after, idx)-1]

def main():
    node_speeds = []
    cuda_speeds = []
    result = {}
    for t in range(4, 11): # number of threads
        with open(f'results/result-{2**t}.pkl', 'rb') as inp:
            result = pickle.load(inp)
    
        best_block_size = 0
        best_node_speed = 0
        best_cuda_speed = 0
        for b in [1,2,4,8]: # block size
            speed = result[(2**t, b)][4] / result[(2**t, b)][3]
            cuda_speed = result[(2**t, b)][4] / result[(2**t, b)][1]
            if speed > best_node_speed:
                best_node_speed = speed
                best_block_size = b
            if cuda_speed > best_cuda_speed:
                best_cuda_speed = cuda_speed
            
        node_speeds.append(best_node_speed)
        cuda_speeds.append(best_cuda_speed)
        print(best_block_size)
    
    fig, (ax1,ax2) = plt.subplots(2,1)

    line, = ax1.plot([2**i for i in range(4,11)], node_speeds)

    ax1.set_xscale('log', base=2)
    ax1.set_title("Speed with respect to CUDA + Nearest Neighbors")
    ax1.set_xlabel('threads')
    ax1.set_ylabel('nodes added per second')


    line, = ax2.plot([2**i for i in range(4,11)], cuda_speeds)

    ax2.set_xscale('log', base=2)
    ax2.set_title("Speed with respect to only CUDA")
    ax2.set_xlabel('threads')
    ax2.set_ylabel('nodes added per second')



    fig.tight_layout(pad=2)

    plt.show()
            

                



if __name__ == "__main__":
    main()