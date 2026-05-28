# JRRA-for-D2D-Networks

**Official C++ Implementation for the Journal Paper:** *"Block coordinate descent for joint delay-energy optimization in multi-hop D2D networks"*

This repository contains an industrial-grade, high-performance C++ optimization engine designed for Joint Routing and Resource Allocation (JRRA) in Device-to-Device (D2D) networks. The core solver relies on the Block Coordinate Descent (BCD) framework to efficiently decouple and solve complex, multi-constraint mathematical models.

The engine features a suite of advanced mathematical solvers, including the **Frank-Wolfe (FW) algorithm**, **Lagrangian Dual Decomposition (LDD)**, a **Game-Theoretic Greedy Baseline (GGB)**, and a highly optimized **Primal-Dual Interior Point Method (IPM)** utilizing block-decoupled Sherman-Morrison Hessian inversions.

## Key Engineering Features

* **Robust Numerical Stability:** Implements Log-Sum-Exp (LSE) smoothing, strict Euclidean simplex projections, and adaptive barrier centering to prevent gradient vanishing/exploding.
* **Memory-Safe Architecture:** Eliminates dynamic heap fragmentation during high-frequency routing queries (DFS/SPFA) via contiguous `std::vector` memoization and `.noalias()` Eigen evaluations.
* **ABI-Safe Data I/O:** Utilizes low-level C-style I/O (`fopen_s`, `sscanf_s`) for topology parsing to bypass STL ABI incompatibilities across cross-platform compilation targets.
* **Physical Law Verification:** Features a strict "Scale-Down Mechanism" and a dual-track delay audit engine to ensure all mathematical expectations strictly adhere to Shannon Capacity physical bounds.

## Prerequisites and Dependencies

This project is built for high-performance scientific computing and requires modern C++ standards (C++14/17):

* **Compiler/IDE:** Visual Studio 2019 (v142 toolset) or any C++17 compliant compiler (GCC/Clang).
* **Math Library:** [Eigen 3](http://eigen.tuxfamily.org/) (specifically `<Eigen/Dense>` and `<Eigen/Sparse>`) for high-speed matrix operations and LDLT decompositions.

## Core Modules and File Structure

| File / Module | Description |
| --- | --- |
| `main.cpp` | Main execution engine orchestrating batch experiments, baseline comparisons, and Scenario D Micro-level Topology Case Studies. |
| `graph.h` & `graph.cpp` | Core graph topology definitions, robust Yen's K-Shortest Paths (KSP), SPFA cycle-immune delay auditing, and dynamic-weight Dijkstra oracles. |
| `optimization_solvers.h` | Unified API header for all optimization solvers (`FW`, `LDD`, `GGB`, `SCA`). Includes the `OptimizationResult` standardized telemetry struct. |
| `optimization_solvers.cpp` | Implements the Frank-Wolfe gradient routing, Dual Decomposition resource allocation, and Game-Theoretic localized power swapping. |
| `ipm_solver.h` & `.cpp` | High-performance Primal-Dual Interior Point Method solver featuring Max-Bottleneck fractional path extraction and exact Newton descent. |
| `initialization.cpp` | Intelligent warm-start strategies (`KSP Uniform`, `KSP Dual-Warm`, `GGB Expansion`) and strict $\epsilon$-Keep-Alive resource allocations. |
| `evaluation.cpp` | Micro-level telemetry probes and CSV exporters for detailed commodity path analysis and edge-level delay/power inspection. |

## Build and Execution

Compilation instructions for Visual Studio 2019:

1. Clone this repository to your local machine.
2. Open Visual Studio 2019 and create a new C++ Console Application project.
3. Import all `.h` and `.cpp` files into the project.
4. **Configure Eigen:** Right-click your project -> `Properties` -> `C/C++` -> `General` -> `Additional Include Directories`, and add the path to your local Eigen library folder.
5. Ensure the Platform Toolset is set to Visual Studio 2019 (v142) and the Language Standard is set to **C++17**.
6. Build the solution in **Release mode** (Critical for activating Eigen's vectorization and loop unrolling for maximum execution speed).

## Input Data Formats

The optimization engine requires network topology and traffic demand data to be supplied in standard CSV format. The robust `Col_gen` module parses these files natively.

### 1. Graph File (`graph_file.csv`)

Defines the physical network topology and edge weights (distances in meters).

```csv
id,source,target,distance
0,1,2,15.4321
1,2,3,10.0000

```

### 2. Demand File (`demand_file.csv`)

Defines the multi-commodity traffic requirements between D2D pairs.

```csv
id,source,target,demand
0,1,5,5.0000
1,3,7,12.5000

```

## License and Citation

*(Placeholder: If this code is used for academic purposes, please cite the corresponding journal paper once published.)*
