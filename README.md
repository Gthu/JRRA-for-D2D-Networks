# JRRA-for-D2D-Networks

Official C++ Implementation for the Journal Paper: "Joint Routing and Resource Allocation for Energy-Aware Min-Max Delay Optimization in Multi-Hop D2D Networks"

This repository contains the high-performance C++ optimization engine designed for Joint Resource Routing and Allocation (JRRA) in Device-to-Device (D2D) networks. The core solver relies on the Block Coordinate Descent (BCD) framework and the Frank-Wolfe (FW) algorithm to efficiently solve complex, multi-constraint mathematical models, including joint power and bandwidth allocation based on Shannon capacity derivations.

## Prerequisites and Dependencies

This project is built for high-performance scientific computing and relies on the following environments and libraries:

- Compiler/IDE: Visual Studio 2019 (v142 toolset)
- Math Library: Eigen3 (specifically <Eigen/Dense>) for high-speed matrix and linear algebra operations
- Standard Libraries: <chrono>, <iostream>, <stack>, <cstdio>, <cmath>, <iomanip>, <queue> (for BFS connectivity), and <fstream> (for CSV report generation)

## Core Modules and File Structure

| File / Module | Description |
| --- | --- |
| main_joint_opt.cpp | Main execution engine orchestrating the BCD experiments, initialization, solver routines, and metric outputs |
| graph.h | Defines core graph data structures, BFS connectivity checks, and routing algorithms (e.g., K-Shortest Paths, Dijkstra) |
| calculate_rates.cpp | Implements Shannon capacity calculations based on physical link parameters, bandwidth, and power distribution |
| initialize_resources.cpp | Handles the baseline initialization for nodal powers, active links, and bandwidth allocations before optimization |
| solver_frank_wolfe.cpp | Implementation of the Frank-Wolfe algorithm for multi-commodity network flow (MCNF) and gradient-based routing |
| solver_dual.h / .cpp | Core definitions and implementations for dual-based optimization algorithms |
| ipm_solver.h | Interface and structures for the Interior Point Method (IPM) solver |

## Build and Execution

Compilation instructions for Visual Studio 2019:

1. Clone this repository to your local machine.
2. Open Visual Studio 2019 and create a new C++ Console Application project.
3. Import all .h and .cpp files into the project.
4. Configure Eigen: Right-click your project -> Properties -> C/C++ -> General -> Additional Include Directories, and add the path to your local Eigen library folder.
5. Ensure the Platform Toolset is set to Visual Studio 2019 (v142).
6. Build the solution in Release mode (recommended for optimization algorithms to ensure maximum execution speed).

## Input Data Formats

The optimization engine requires network topology and traffic demand data to be supplied in standard CSV format. The system parses these files during the initialization phase.

1. Graph File (graph_file.csv)
Defines the physical network topology and edge weights (e.g., distances or propagation delays).

id,source,target,distance
0,1,2,15.4321
1,2,3,10.0000

2. Demand File (demand_file.csv)
Defines the traffic requirements between D2D pairs.

id,source,target,demand
0,1,5,5.0000
1,3,7,12.5000

## License and Citation

(Placeholder: If this code is used for academic purposes, please cite the corresponding journal paper once published.)
