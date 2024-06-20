NVCC = /usr/bin/nvcc

.PHONY: rrt

rrt:
	$(NVCC) cudaRRT.cu draw.cpp  main.cpp  rrt.cpp  scene.cpp  solver.cpp cudaRRTSolver.cpp -o rrt -I ~/.local/usr/include/jsoncpp/ ~/.local/usr/lib/x86_64-linux-gnu/libjsoncpp.a -I ./CImg-3.4.0_pre06092412 -I ./flann-1.8.4-src/src/cpp -lm -lpthread -lX11 -I ./CXXGraph-3.1.0/include/CXXGraph -I ./CXXGraph-3.1.0/include

clean:
	rm -rf *.o *.a vecadd
