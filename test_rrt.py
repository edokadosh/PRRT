import time
from subprocess import Popen, PIPE
from tqdm import tqdm
import pickle

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


batch = 10

def get_res(s, before, after):
    idx = s.find(before) + len(before)+1
    return s[idx:s.find(after, idx)-1]

def main():
    result = {}
    for t in range(4, 12): # number of threads
        for b in [1,2,4,8]: # block size
            print("-- testing: threads =", 2**t, ", block size =", b, "--")
            failed = 0
            sum_cuda_time = 0
            sum_tree_time = 0
            sum_time = 0
            sum_nodes = 0
            for i in tqdm(range(batch)): # batch of tests
                process = Popen(['./rrt', '-scene', 'scenes/generated_big_scene_2.json', '-solver', 'cuda', '-seed', str(1234+i), '-num_threads', str(2**t), '-block_size', str(b), '-q'], stdout=PIPE, stderr=PIPE)
                stdout, stderr = process.communicate()
                if wait_timeout(process, 200) is None:
                    failed+=1
                    tqdm.write("failed")
                else:
                    stdout = stdout.decode("utf-8")

                    cuda_time = float(get_res(stdout, "Solving on cuda took", "seconds"))

                    tree_time = float(get_res(stdout, "Combining trees took", "seconds"))

                    time = float(get_res(stdout, "Time elapsed:", "seconds"))

                    nodes = int(get_res(stdout, "Solved!", "nodes"))

                    sum_time += time
                    sum_cuda_time += cuda_time
                    sum_tree_time += tree_time
                    sum_nodes += nodes

                    tqdm.write("cuda time: " + str(cuda_time) + " tree time: " + str(tree_time) + " time: " + str(time) + " nodes: " + str(nodes))
            
            result[(2**t,b)] = (failed, sum_cuda_time/(batch-failed), sum_tree_time/(batch-failed), sum_time/(batch-failed), sum_nodes/(batch-failed))

            with open(f'result-{2**t}.pkl', 'wb') as outp:
                pickle.dump(result, outp, pickle.HIGHEST_PROTOCOL) 
                
    print("---result---")
    print(result)

    with open('result.pkl', 'wb') as outp:
        pickle.dump(result, outp, pickle.HIGHEST_PROTOCOL)

                



if __name__ == "__main__":
    main()