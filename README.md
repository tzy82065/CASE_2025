## PIBT_T
Code and supplementary files for CASE_2025

This respository consists of a code pack including supplementary files `PIBT_T/supplementary_file` for paper "Fast Multi-Agent Path Planning with Turn Actions: A Priority Inheritance Approach", which is submitted to the 2025 IEEE 21st International Conference on Automation Science and Engineering (CASE 2025).

## Author, Copyright and Acknowledgement
The code is modified on Okumura's implementation of PIBT (https://github.com/Kei18/pibt2/). This is the original PIBT, which exhibits significant high-speed and scalability in classic MAPF and MAPD problems.

Please cite the following paper if you use the code in your published research:
Okumura, Keisuke, et al. "Priority inheritance with backtracking for iterative multi-agent path finding." Artificial Intelligence 310 (2022):103752.
...

## Description
- Our algorithm adapts PIBT to MAPF with turn actions.
- It is implemented in C++ with CMake(≥v3.16).
- The following files in pibt2 (https://github.com/Kei18/pibt2/) are modified:
orientation.cpp/.hpp, pibt.cpp/.hpp, plan.cpp/.hpp, problem.cpp/.hpp, solver.cpp/.hpp

## Building
```sh
git clone --recursive https://github.com/tzy82065/CASE_2025.git
cd PIBT_T
mkdir build && cd build
cmake ..
make
```

## Usage
```sh
./mapf -i ../instances/mapf/sample.txt -s PIBT -o result.txt -v
```

## Experiment Setup
- The experiment is conducted on 5 MAPF Benchmark maps: empty-32-32, random-32-32-20, room-64-64-8, warehouse-10-20-10-2-2, den520d. Map files `.map` and corresponding scenario files `.scen` can be download on https://movingai.com/benchmarks/mapf/index.html.
- All agents start planning with an orientation "North".

## Visualize
A lot of Thanks to Okumura again! The visualizer module from him can be download in (https://github.com/kei18/mapf-visualizer).
