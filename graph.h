#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdint>
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
// 1. Core Types and Constants
// ==========================================

constexpr double kEpsilon = 1e-5;

using weight_t = double;
using GraphPath = std::vector<std::size_t>;
using Link = std::pair<int, int>;
using LinkMap = std::map<Link, double>;
using FlowSolution = std::map<std::size_t, LinkMap>;
using CostSolution = std::map<std::size_t, LinkMap>;

// ==========================================
// 2. Data Structures
// ==========================================

/**
 * @brief Represents the initial resource allocation for the network model.
 */
struct InitialAllocation {
    std::set<Link> active_edges;
    LinkMap initial_power;
    LinkMap initial_bandwidth;
};

/**
 * @brief Multi-Commodity Network Flow (MCNF) optimization solution state.
 */
struct MCNFSolution {
    FlowSolution flow_routing;
    LinkMap dual_variables;
    bool success = false;
};

/**
 * @brief Results from the subgradient descent optimization phase.
 */
struct SubgradientResult {
    FlowSolution optimal_flow;
    std::vector<double> optimal_gamma;
    std::vector<double> optimal_mu;
    bool converged;
};

/**
 * @brief Configuration parameters for the Frank-Wolfe (Conditional Gradient) algorithm.
 */
struct FWConfig {
    double penalty_multiplier;
    double step_size_alpha;
    double convergence_tolerance;
    double gradient_epsilon;
    int max_iterations;
};

// ==========================================
// 3. Data Loading Module (Col_gen)
// ==========================================

/**
 * @brief Network topology and delay data generator/loader.
 * * Parses CSV-based network edge data and constructs the baseline 
 * node properties and delay adjacency matrices.
 */
class Col_gen {
public:
    std::vector<std::tuple<std::size_t, std::size_t, weight_t>> edge_list;
    std::size_t nodes_num = 0;
    std::vector<std::vector<weight_t>> adjacency_matrix_dl;

    /**
     * @brief Constructs the column generator and loads topology from a file.
     * @param file_path Absolute or relative path to the network topology CSV.
     * @note File I/O utilizes low-level `fopen_s` for memory safety and strict POSIX compliance.
     * Time complexity for graph construction is O(E), where E is the number of edges.
     */
    explicit Col_gen(const std::string& file_path) {
        FILE* file_pointer = nullptr;
        errno_t err = fopen_s(&file_pointer, file_path.c_str(), "r");

        if (err != 0 || file_pointer == nullptr) {
            std::cerr << "[Fatal Error] Failed to open topology file. Code: " << err << std::endl;
            exit(EXIT_FAILURE);
        }

        char buffer[1024];

        // Skip CSV header
        if (!fgets(buffer, sizeof(buffer), file_pointer)) {
            std::cerr << "[Warning] Topology file is empty." << std::endl;
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
            std::size_t source_node, target_node;
            double link_delay;

            int parsed_items = sscanf_s(buffer, "%d %zu %zu %lf", &dummy_id, &source_node, &target_node, &link_delay);
            if (parsed_items < 4) {
                continue;
            }

            edge_list.emplace_back(source_node, target_node, link_delay);

            if (source_node > nodes_num) nodes_num = source_node;
            if (target_node > nodes_num) nodes_num = target_node;
        }

        fclose(file_pointer);

        if (!edge_list.empty()) {
            nodes_num++;
        }

        if (nodes_num > 5000) {
            std::cerr << "[Warning] Graph order (" << nodes_num << ") exceeds allocation safety threshold. Skipping dense matrix allocation." << std::endl;
        } else {
            adjacency_matrix_dl.assign(nodes_num, std::vector<weight_t>(nodes_num, std::numeric_limits<weight_t>::max()));
            for (const auto& edge : edge_list) {
                std::size_t u = std::get<0>(edge);
                std::size_t v = std::get<1>(edge);
                if (u < nodes_num && v < nodes_num) {
                    adjacency_matrix_dl[u][v] = std::get<2>(edge);
                }
            }
        }
    }

    [[nodiscard]] std::size_t get_nodes_num() const { return nodes_num; }
};

// ==========================================
// 4. Graph Topology Module
// ==========================================

/**
 * @brief Lightweight directed graph representation tailored for shortest-path routing algorithms.
 */
class Graph {
public:
    std::vector<std::vector<std::pair<int, double>>> adj;
    std::size_t num_nodes;

    /**
     * @brief Constructs the directed graph from the parsed topology data.
     * @param n Total number of vertices in the graph.
     * @param G Constant reference to the loaded topology generator.
     * @note Constructs adjacency lists in O(E) time to ensure efficient iterations during FW/BCD loops.
     */
    Graph(std::size_t n, const Col_gen& G) : num_nodes(n), adj(n) {
        for (const auto& edge : G.edge_list) {
            std::size_t u = std::get<0>(edge);
            std::size_t v = std::get<1>(edge);
            double w = std::get<2>(edge);

            if (u >= n || v >= n) {
                std::cerr << "[Graph Error] Out-of-bounds edge detected: " << u << "->" << v << ". Max node index is " << n - 1 << ". Edge ignored." << std::endl;
                continue;
            }
            add_directed_edge(u, v, w);
        }
    }

    void add_directed_edge(std::size_t u, std::size_t v, double weight) {
        adj[u].emplace_back(static_cast<int>(v), weight);
    }

    [[nodiscard]] std::size_t size() const { return num_nodes; }
};

// ==========================================
// 5. Core Mathematical Optimization API
// ==========================================

// --- Initialization & Utilities ---

std::vector<std::tuple<std::size_t, std::size_t, double>> Get_demand(const std::string& file_path);

InitialAllocation initialize_resources(
    const Graph& graph,
    const std::set<Link>& active_subgraph,
    const std::vector<double>& node_max_powers,
    double total_spectrum_bandwidth
);

LinkMap calculate_rates(
    const Graph& graph,
    std::size_t num_nodes,
    const std::set<Link>& active_edges,
    const LinkMap& bandwidth_allocations,
    const LinkMap& power_allocations,
    double noise_density_dbm_per_mhz,
    std::size_t edge_size,
    double total_spectrum_bandwidth,
    const std::vector<double>& node_max_powers
);

// --- Routing & Pathfinding (Shortest Path Subproblems) ---

/**
 * @brief Standard Dijkstra's algorithm with optional node restrictions.
 * @param graph The network topology.
 * @param src Source node index.
 * @param dest Destination node index.
 * @param blocked_u Optional constraint to block a specific source link node.
 * @param blocked_v Optional constraint to block a specific target link node.
 * @param ignored_nodes Optional list of nodes to strictly avoid during routing.
 * @return A tuple containing the optimal path and its scalar cost.
 * @note Time Complexity: O((V + E) log V). Forms the linear subproblem oracle for the Frank-Wolfe loop.
 */
std::pair<GraphPath, double> dijkstra(
    const Graph& graph,
    std::size_t src,
    std::size_t dest,
    std::size_t blocked_u = std::numeric_limits<std::size_t>::max(),
    std::size_t blocked_v = std::numeric_limits<std::size_t>::max(),
    const std::vector<std::size_t>& ignored_nodes = {}
);

std::pair<GraphPath, double> dijkstra(
    const Graph& graph,
    std::size_t src,
    std::size_t dest,
    const std::map<Link, double>& dynamic_weights
);

std::vector<GraphPath> k_shortest_paths(
    std::size_t k,
    const GraphPath& target_path,
    std::size_t num_paths,
    const Col_gen& topology_data,
    const Graph& graph,
    std::size_t src
);

std::vector<Link> dijkstra_weighted(
    const Graph& graph,
    std::size_t num_nodes,
    int src,
    int dst,
    const std::map<Link, double>& penalized_weights
);

// --- Evaluation & Solvers ---

std::tuple<double, double, double, std::vector<double>> evaluate_network_state(
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& commodity_demands,
    const std::vector<std::map<Link, double>>& flow_routing_matrix,
    const std::map<Link, double>& rate_capacities,
    const std::map<Link, double>& power_allocations,
    const std::map<Link, double>& bandwidth_allocations,
    double total_bandwidth,
    int num_nodes
);

void export_commodity_path_details(
    const std::string& file_path,
    const std::string& algo_name,
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& commodity_demands,
    const std::vector<std::map<Link, double>>& flow_routing_matrix,
    const std::map<Link, double>& rate_capacities,
    const std::map<Link, double>& power_allocations,
    const std::map<Link, double>& bandwidth_allocations,
    double total_bandwidth
);

/**
 * @brief Executes the Frank-Wolfe (Conditional Gradient) algorithm for network routing optimization.
 * @param graph Underlying graph topology.
 * @param edge_list Master list of network edges.
 * @param num_nodes Total vertices in the graph.
 * @param active_edges Subset of operational links.
 * @param commodity_demands Source-destination traffic matrix.
 * @param initial_rates Baseline capacities of the active edges.
 * @param initial_power Baseline power allocation.
 * @param flow_routing_matrix (In/Out) The decision variables for traffic flow on links.
 * @param config Hyperparameters guiding step size and convergence.
 * @param dynamic_edge_weights Real-time penalties/costs for the linear shortest-path subproblem.
 * @note Computes the descent direction by minimizing the linearized objective (via `dijkstra_weighted`). 
 * Updates the flow routing matrix via a convex combination of the current state and the subproblem solution. 
 * Algorithm converges asymptotically at a rate of O(1/k).
 */
void solve_frank_wolfe(
    const Graph& graph,
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& edge_list,
    std::size_t num_nodes,
    std::set<Link>& active_edges,
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& commodity_demands,
    const LinkMap& initial_rates,
    const LinkMap& initial_power,
    std::vector<std::map<Link, double>>& flow_routing_matrix,
    FWConfig config,
    const std::map<Link, double>& dynamic_edge_weights = {}
);

inline double calculate_l2_norm(const std::vector<double>& vector_a, const std::vector<double>& vector_b) {
    double sum_sq = 0.0;
    for (std::size_t i = 0; i < vector_a.size(); ++i) {
        double diff = vector_a[i] - vector_b[i];
        sum_sq += diff * diff;
    }
    return std::sqrt(sum_sq);
}

// --- Initialization Packagers ---

/**
 * @brief Container packaging all initial states before entering the optimization loops.
 */
struct InitializationResult {
    std::vector<std::map<Link, double>> flow_routing_matrix;     
    std::vector<std::map<Link, double>> initial_ksp_routing; 
    std::set<Link> active_edges;                                  
    LinkMap initial_power;                                         
    LinkMap initial_bandwidth;                                     
    LinkMap initial_rate_capacity;                                 
};

InitializationResult initialize_with_ksp_uniform(
    const Graph& graph, 
    Col_gen topology_data,
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& commodity_demands,
    int num_nodes, 
    double max_power_per_node, 
    double total_spectrum_bandwidth, 
    double noise_density_dbm_per_mhz
);

InitializationResult initialize_with_ksp_warm(
    const Graph& graph, 
    Col_gen topology_data,
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& commodity_demands,
    int num_nodes, 
    double max_power_per_node, 
    double user_alpha, 
    double total_spectrum_bandwidth, 
    double noise_density_dbm_per_mhz
);

InitializationResult initialize_with_ksp_game(
    const Graph& graph, 
    const Col_gen& topology_data, 
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& commodity_demands,
    int num_nodes, 
    double max_power_per_node, 
    double user_alpha, 
    double total_spectrum_bandwidth, 
    double noise_density_dbm_per_mhz
);

InitializationResult initialize_with_ggb_warm_start(
    const Graph& graph, 
    Col_gen topology_data,
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& commodity_demands,
    int num_nodes, 
    double max_power_per_node, 
    double total_spectrum_bandwidth, 
    double noise_density_dbm_per_mhz
);
