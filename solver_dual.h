#ifndef OPTIMIZATION_SOLVERS_H_
#define OPTIMIZATION_SOLVERS_H_

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "graph.h"

// =========================================================
// Optimization Results Data Structure
// =========================================================

/**
 * @brief Encapsulates the network state and performance metrics post-optimization.
 */
struct OptimizationResult {
    std::vector<double> optimal_bandwidths;       
    std::vector<double> optimal_powers;           
    std::vector<double> optimal_rates;            
    std::vector<double> optimal_transmission_times; 

    // System-level Performance Metrics
    double max_end_to_end_delay;                  
    double total_system_energy;                   
    
    std::vector<double> commodity_max_delays;     
    double max_expected_sum_delay;                
    
    std::vector<std::map<Link, double>> final_flow_routing; 

    OptimizationResult() 
        : max_end_to_end_delay(0.0), total_system_energy(0.0), max_expected_sum_delay(0.0) {}

    OptimizationResult(std::vector<double> bandwidths, 
                       std::vector<double> powers, 
                       std::vector<double> rates, 
                       std::vector<double> times)
        : optimal_bandwidths(std::move(bandwidths)), 
          optimal_powers(std::move(powers)), 
          optimal_rates(std::move(rates)), 
          optimal_transmission_times(std::move(times)), 
          max_end_to_end_delay(0.0), 
          total_system_energy(0.0),
          max_expected_sum_delay(0.0) {}
};

// =========================================================
// Optimization Solvers API
// =========================================================

/**
 * @brief Executes a distributed resource allocation solver utilizing Lagrangian Dual Decomposition.
 * * @param edge_list Master list of network edges (source, destination, delay).
 * @param num_nodes Total number of vertices in the network graph.
 * @param node_max_powers Vector specifying the maximum power constraint per node.
 * @param total_bandwidth_mhz The total available spectrum bandwidth in MHz.
 * @param commodity_demands Traffic demand matrix specifying source, destination, and volume.
 * @param current_flow_routing The pre-computed flow routing matrix on active links.
 * @param noise_density_dbm_per_mhz Additive White Gaussian Noise (AWGN) power spectral density.
 * @param fairness_alpha The scaling parameter for the $\alpha$-fairness utility function.
 * @param convergence_epsilon Tolerance threshold for subgradient descent termination.
 * * @return OptimizationResult containing optimal bandwidth/power matrices and system metrics.
 * * @note The algorithm solves the network utility maximization (NUM) problem by relaxing 
 * coupling constraints via Lagrangian multipliers. It iteratively updates primal variables 
 * (power, bandwidth) and dual variables (prices) using subgradient descent. 
 * Convergence rate is roughly $\mathcal{O}(1/\sqrt{k})$ where $k$ is the iteration count.
 */
OptimizationResult run_dual_decomposition_solver(
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& edge_list,
    int num_nodes,
    const std::vector<double>& node_max_powers,
    double total_bandwidth_mhz,
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& commodity_demands,
    const std::vector<std::map<Link, double>>& current_flow_routing,
    double noise_density_dbm_per_mhz,
    double fairness_alpha,
    double convergence_epsilon
);

/**
 * @brief Executes a Game-Theoretic Greedy allocation strategy (Resource Allocation ONLY).
 * * @param edge_list Master list of network edges.
 * @param num_nodes Total number of vertices in the network graph.
 * @param node_max_powers Vector specifying the maximum power constraint per node.
 * @param total_bandwidth_mhz The total available spectrum bandwidth in MHz.
 * @param commodity_demands Traffic demand matrix.
 * @param current_flow_routing The active routing baseline to optimize resources over.
 * @param noise_density_dbm_per_mhz AWGN power spectral density.
 * * @return OptimizationResult containing the resource allocation mapping.
 * * @note This solver models the resource contention as a non-cooperative game, allocating 
 * marginal resources sequentially to links that yield the steepest gradient in system utility. 
 * It approximates a Nash Equilibrium under localized information constraints.
 */
OptimizationResult run_game_greedy_allocation(
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& edge_list,
    int num_nodes,
    const std::vector<double>& node_max_powers,
    double total_bandwidth_mhz,
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& commodity_demands,
    const std::vector<std::map<Link, double>>& current_flow_routing,
    double noise_density_dbm_per_mhz
);

/**
 * @brief Executes a static, uniform equal-allocation baseline for benchmarking.
 * * @param edge_list Master list of network edges (utilizing generic weight_t).
 * @param commodity_demands Traffic demand matrix.
 * @param current_flow_routing The pre-computed flow routing matrix.
 * @param total_bandwidth_mhz The total available spectrum bandwidth in MHz.
 * @param max_power_per_node A uniform maximum power constraint applied to all nodes.
 * @param noise_density_dbm_per_mhz AWGN power spectral density.
 * * @return OptimizationResult representing the uniform heuristic allocation.
 * * @note Time complexity is $\mathcal{O}(|E|)$ where $|E|$ is the number of active edges. 
 * Operates purely as a lower-bound performance baseline without iterative optimization.
 */
OptimizationResult run_static_equal_allocation(
    const std::vector<std::tuple<std::size_t, std::size_t, weight_t>>& edge_list,
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& commodity_demands,
    const std::vector<std::map<Link, double>>& current_flow_routing,
    double total_bandwidth_mhz,
    double max_power_per_node,
    double noise_density_dbm_per_mhz
);

/**
 * @brief Executes the combined Game-Theoretic Greedy baseline (Routing + Resource Allocation).
 * * @param edge_list Master list of network edges.
 * @param num_nodes Total number of vertices in the network graph.
 * @param node_max_powers Vector specifying the maximum power constraint per node.
 * @param total_bandwidth_mhz The total available spectrum bandwidth in MHz.
 * @param commodity_demands Traffic demand matrix.
 * @param noise_density_dbm_per_mhz AWGN power spectral density.
 * * @return OptimizationResult containing both computed routing paths and resource allocations.
 */
OptimizationResult run_game_greedy_baseline(
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& edge_list,
    int num_nodes,
    const std::vector<double>& node_max_powers,
    double total_bandwidth_mhz,
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& commodity_demands,
    double noise_density_dbm_per_mhz
);

/**
 * @brief Executes the Successive Convex Approximation (SCA) solver baseline.
 * * @param edge_list Master list of network edges.
 * @param num_nodes Total number of vertices in the network graph.
 * @param node_max_powers Vector specifying the maximum power constraint per node.
 * @param total_bandwidth_mhz The total available spectrum bandwidth in MHz.
 * @param commodity_demands Traffic demand matrix.
 * @param noise_density_dbm_per_mhz AWGN power spectral density.
 * * @return OptimizationResult derived from the SCA algorithm.
 * * @note Handles non-convex Shannon capacity constraints $\log_2(1 + \text{SINR})$ by applying 
 * a first-order Taylor expansion around a local operating point. Iteratively solves the 
 * resulting convexified subproblems until KKT conditions are met.
 */
OptimizationResult run_sca_baseline(
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& edge_list,
    int num_nodes,
    const std::vector<double>& node_max_powers,
    double total_bandwidth_mhz,
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& commodity_demands,
    double noise_density_dbm_per_mhz
);

#endif  // OPTIMIZATION_SOLVERS_H_
