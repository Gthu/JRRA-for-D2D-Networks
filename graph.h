#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

// ==========================================
// 1. Fundamental Types & Constants
// ==========================================

constexpr double kEpsilon = 1e-5;

using WeightType = double;
using GraphPath = std::vector<size_t>;
using Link = std::pair<int, int>; 
using LinkMap = std::map<Link, double>;
using FlowSolution = std::map<size_t, LinkMap>;
using CostSolution = std::map<size_t, LinkMap>;

// ==========================================
// 2. Data Structures
// ==========================================

/**
 * @brief Represents the initial resource allocation for the network.
 */
struct InitialAllocation {
    std::set<Link> active_edges;
    LinkMap initial_powers;
    LinkMap initial_bandwidths;
};

/**
 * @brief Captures the output of the Multi-Commodity Network Flow (MCNF) solver.
 */
struct McnfSolution {
    FlowSolution flow_allocation;
    LinkMap dual_variables;
    bool is_successful = false;
};

/**
 * @brief Results from the Subgradient optimization step.
 */
struct SubgradientResult {
    FlowSolution optimal_flows;
    std::vector<double> optimal_gammas;
    std::vector<double> optimal_mus;
    bool has_converged = false;
};

/**
 * @brief Configuration parameters for the Frank-Wolfe optimization algorithm.
 */
struct FrankWolfeConfig {
    double penalty_multiplier;
    double step_size_alpha;
    double convergence_tolerance;
    double gradient_epsilon;
    int max_iterations;
};

// ==========================================
// 3. Column Generation (Topology Reader)
// ==========================================

/**
 * @brief Handles graph topology parsing and memory allocation for adjacency matrices.
 * @note Instantiating this class performs an O(E) file parsing operation. Matrix 
 * allocation takes O(V^2) spatial complexity, bounded by a 5000-node safety threshold.
 */
class ColumnGenerator {
public:
    std::vector<std::tuple<size_t, size_t, WeightType>> edge_list;
    size_t num_nodes = 0;
    std::vector<std::vector<WeightType>> delay_adjacency_matrix;

    /**
     * @brief Constructs the topology from a CSV data file.
     * @param file_path Absolute or relative path to the topology CSV.
     * @throws std::runtime_error If the file cannot be opened.
     */
    explicit ColumnGenerator(const std::string& file_path) {
        FILE* file_pointer = nullptr;
#ifdef _MSC_VER
        errno_t err = fopen_s(&file_pointer, file_path.c_str(), "r");
        if (err != 0 || file_pointer == nullptr) {
            throw std::runtime_error("[ColumnGenerator] IO Error: Cannot open file.");
        }
#else
        file_pointer = fopen(file_path.c_str(), "r");
        if (file_pointer == nullptr) {
            throw std::runtime_error("[ColumnGenerator] IO Error: Cannot open file.");
        }
#endif

        char buffer[1024];

        if (!fgets(buffer, sizeof(buffer), file_pointer)) {
            fclose(file_pointer);
            return;
        }

        while (fgets(buffer, sizeof(buffer), file_pointer)) {
            for (int i = 0; buffer[i]; ++i) {
                if (buffer[i] == ',') {
                    buffer[i] = ' ';
                }
            }

            int dummy_id;
            size_t source_node;
            size_t target_node;
            double delay_weight;

#ifdef _MSC_VER
            int parsed_count = sscanf_s(buffer, "%d %zu %zu %lf", &dummy_id, &source_node, &target_node, &delay_weight);
#else
            int parsed_count = sscanf(buffer, "%d %zu %zu %lf", &dummy_id, &source_node, &target_node, &delay_weight);
#endif

            if (parsed_count < 4) {
                continue;
            }

            edge_list.emplace_back(source_node, target_node, delay_weight);

            num_nodes = std::max({num_nodes, source_node, target_node});
        }

        fclose(file_pointer);

        if (!edge_list.empty()) {
            num_nodes++;
        }

        if (num_nodes <= 5000) {
            delay_adjacency_matrix.assign(num_nodes, std::vector<WeightType>(num_nodes, std::numeric_limits<WeightType>::max()));
            for (const auto& edge : edge_list) {
                size_t u = std::get<0>(edge);
                size_t v = std::get<1>(edge);
                if (u < num_nodes && v < num_nodes) {
                    delay_adjacency_matrix[u][v] = std::get<2>(edge);
                }
            }
        }
    }

    [[nodiscard]] size_t get_num_nodes() const { return num_nodes; }
};

// ==========================================
// 4. Graph Architecture
// ==========================================

/**
 * @brief Core graph representation utilizing an adjacency list.
 */
class Graph {
public:
    std::vector<std::vector<std::pair<int, double>>> adjacency_list;
    size_t num_nodes;

    /**
     * @brief Constructs the routing graph directly from the ColumnGenerator's edge list.
     * @param total_nodes Upper bound of nodes to allocate for the adjacency list.
     * @param col_gen Reference to the populated ColumnGenerator instance.
     */
    Graph(size_t total_nodes, const ColumnGenerator& col_gen) : num_nodes(total_nodes), adjacency_list(total_nodes) {
        for (const auto& edge : col_gen.edge_list) {
            size_t u = std::get<0>(edge);
            size_t v = std::get<1>(edge);
            double weight = std::get<2>(edge);

            if (u >= total_nodes || v >= total_nodes) {
                continue;
            }
            add_directed_edge(u, v, weight);
        }
    }

    /**
     * @brief Inserts a directed edge into the adjacency list.
     * @param u Source node index.
     * @param v Destination node index.
     * @param weight Edge weight (e.g., propagation delay).
     */
    void add_directed_edge(size_t u, size_t v, double weight) {
        adjacency_list[u].emplace_back(static_cast<int>(v), weight);
    }

    [[nodiscard]] size_t size() const { return num_nodes; }
};

// ==========================================
// 5. Core Algorithmic Declarations
// ==========================================

/**
 * @brief Parses the traffic demand matrix from disk.
 * @param file_path Path to the demand matrix data.
 * @return A vector of tuples containing (Source, Destination, Required_Bandwidth).
 */
std::vector<std::tuple<size_t, size_t, double>> get_demand(const std::string& file_path);

/**
 * @brief Initializes baseline network resources for mathematical optimization.
 * @param graph The underlying network topology.
 * @param subgraph_edges Active link subset to evaluate.
 * @param nodal_powers Initial transmit power distribution.
 * @param total_spectrum_bandwidth Total available spectrum block.
 * @return Struct containing the baseline active links, powers, and bandwidths.
 */
InitialAllocation initialize_resources(
    const Graph& graph,
    const std::set<Link>& subgraph_edges,
    const std::vector<double>& nodal_powers,
    double total_spectrum_bandwidth
);

/**
 * @brief Calculates Shannon capacity rates based on physical link parameters.
 * @note Time Complexity: O(|E|). Applies logarithmic capacity bound derivations.
 * @param graph The network topology.
 * @param num_nodes Total vertices.
 * @param active_edges Subset of links currently carrying traffic.
 * @param bandwidth_allocation Current bandwidth map.
 * @param power_allocation Current power distribution map.
 * @param noise_density_dbm Thermal noise density.
 * @param edge_size Number of active connections.
 * @param total_spectrum_bandwidth Global system bandwidth.
 * @param max_nodal_powers Vector of physical power limits per node.
 * @return A map of feasible transmission rates per active link.
 */
LinkMap calculate_rates(
    const Graph& graph,
    size_t num_nodes,
    const std::set<Link>& active_edges,
    const LinkMap& bandwidth_allocation,
    const LinkMap& power_allocation,
    double noise_density_dbm,
    size_t edge_size,
    double total_spectrum_bandwidth,
    const std::vector<double>& max_nodal_powers
);

/**
 * @brief Executes standard Dijkstra's algorithm for baseline pathfinding.
 * @note Time Complexity: O(|E| + |V|log|V|). Uses a standard binary heap.
 * @param graph The network topology.
 * @param src Source node.
 * @param dest Destination node.
 * @param blocked_u Optional constraint to sever link (u,v).
 * @param blocked_v Optional constraint to sever link (u,v).
 * @param ignored_nodes Sub-graph node exclusion list.
 * @return A pair comprising the shortest path sequence and the cumulative path cost.
 */
std::pair<GraphPath, double> dijkstra(
    const Graph& graph,
    size_t src,
    size_t dest,
    size_t blocked_u = std::numeric_limits<size_t>::max(),
    size_t blocked_v = std::numeric_limits<size_t>::max(),
    const std::vector<size_t>& ignored_nodes = {}
);

/**
 * @brief Dijkstra's algorithm override supporting dynamically injected weights.
 */
std::pair<GraphPath, double> dijkstra(
    const Graph& graph,
    size_t src,
    size_t dest,
    const std::map<Link, double>& dynamic_weights
);

/**
 * @brief Computes Yen's K-Shortest Paths.
 * @note Used predominantly for path dictionary initialization prior to Flow optimizations.
 * @param k Number of target paths.
 * @param core_path Initial seed path.
 * @param num_iterations Depth of deviation tracking.
 * @param col_gen Column generation data reference.
 * @param graph Target topology.
 * @param src Source vertex.
 * @return Vector of valid GraphPaths.
 */
std::vector<GraphPath> k_shortest_paths(
    size_t k,
    const GraphPath& core_path,
    size_t num_iterations,
    const ColumnGenerator& col_gen,
    const Graph& graph,
    size_t src
);

/**
 * @brief Robust, weighted Dijkstra explicitly mapped for Frank-Wolfe subproblems.
 * @param graph Target topology.
 * @param num_nodes Total system vertices.
 * @param src Source node.
 * @param dst Destination node.
 * @param weights Dynamic, iteration-specific gradient link weights.
 * @return Sequence of traversed links representing the shortest path.
 */
std::vector<Link> dijkstra_weighted(
    const Graph& graph,
    size_t num_nodes,
    int src,
    int dst,
    const std::map<Link, double>& weights
);

/**
 * @brief Solves the multi-commodity routing problem via the Frank-Wolfe algorithm.
 * @note Time Complexity: O(K * (|E| + |V|log|V|)) per iteration. 
 * Converges at a rate of O(1/k) to the optimal fractional flow boundary.
 * @param graph Fixed network topology.
 * @param edge_list Structural edge definitions.
 * @param num_nodes Total vertices.
 * @param active_edges Subset of operational links.
 * @param demand_data Traffic matrices mapping source to destination loads.
 * @param initial_rates Pre-calculated Shannon capacities.
 * @param initial_powers Pre-calculated nodal power states.
 * @param flow_solution [Out] Matrix capturing converged link flows.
 * @param config Frank-Wolfe hyper-parameters (alpha, epsilon, max_iter).
 * @param dynamic_edge_weights Iteration-specific external weight perturbations.
 */
void solve_frank_wolfe(
    const Graph& graph,
    const std::vector<std::tuple<size_t, size_t, double>>& edge_list,
    size_t num_nodes,
    std::set<Link>& active_edges,
    const std::vector<std::tuple<size_t, size_t, double>>& demand_data,
    const LinkMap& initial_rates,
    const LinkMap& initial_powers,
    std::vector<std::map<Link, double>>& flow_solution,
    FrankWolfeConfig config,
    const std::map<Link, double>& dynamic_edge_weights = {}
);

/**
 * @brief Computes the Euclidean (L2) norm difference between two state vectors.
 * @param a First numerical state vector.
 * @param b Second numerical state vector.
 * @return The scalar L2 distance.
 */
inline double calculate_l2_norm(const std::vector<double>& a, const std::vector<double>& b) {
    double sum_squared = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        double difference = a[i] - b[i];
        sum_squared += difference * difference;
    }
    return std::sqrt(sum_squared);
}
