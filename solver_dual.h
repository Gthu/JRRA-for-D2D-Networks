#pragma once

#include <vector>
#include <tuple>
#include <map>
#include <string>

#include "graph.h" 

/**
 * @brief Structure to store the converged physical layer resource allocation and routing metrics.
 * 
 * @note This struct acts as the standard payload returned by all optimization sub-solvers 
 * (Dual Decomposition, Greedy, SCA, and Static Baseline) to the main macro-loop.
 */
struct OptimizationResult {
    // Physical Resource Allocations
    std::vector<double> l_opt;  ///< Optimized bandwidth allocation per link (MHz)
    std::vector<double> p_opt;  ///< Optimized power allocation per link (Watts)
    std::vector<double> r_opt;  ///< Optimized physical transmission rate per link (Mbps)
    std::vector<double> t_opt;  ///< Optimized transmission delay per link (s)

    // System-Level Performance Metrics
    double max_delay;           ///< Maximum end-to-end topological delay across all flows (s)
    double total_energy;        ///< Total energy consumption of the system (Joules)
    double max_sum_delay;       ///< Maximum expected aggregated delay across all flows (s)
    
    std::vector<double> flow_max_delays;           ///< Specific maximum delay mapped to each individual traffic flow
    std::vector<std::map<Link, double>> final_x_flow; ///< Final converged routing fractions (Used primarily by Joint Solvers)

    /**
     * @brief Default constructor initializing performance metrics to zero.
     */
    OptimizationResult() : max_delay(0.0), total_energy(0.0), max_sum_delay(0.0) {}

    /**
     * @brief Parameterized constructor for direct state assignment.
     */
    OptimizationResult(std::vector<double> l, std::vector<double> p, std::vector<double> r, std::vector<double> t)
        : l_opt(std::move(l)), p_opt(std::move(p)), r_opt(std::move(r)), t_opt(std::move(t)), 
          max_delay(0.0), total_energy(0.0), max_sum_delay(0.0) {}
};

// ==================================================================================
// Sub-Solver Interface Definitions
// ==================================================================================

/**
 * @brief Executes the Lagrangian Dual Decomposition solver for joint resource allocation.
 * 
 * @param edge_list Tuple list defining the physical topology (u, v, distance).
 * @param num_nodes Total number of nodes in the network.
 * @param max_node_powers Maximum power budget constraint for each node.
 * @param total_bandwidth_mhz Total available system spectrum bandwidth in MHz.
 * @param traffic_demands Tuple list defining commodity traffic flows (src, dst, volume).
 * @param flow_fractions Routing fractions obtained from the preceding routing phase.
 * @param noise_power_dbm_mhz Noise power spectral density (dBm/MHz).
 * @param alpha_tradeoff Trade-off parameter balancing delay vs. energy consumption.
 * @param tolerance_epsilon Small constant for numerical stability and subgradient truncation.
 * @return OptimizationResult Structure containing the converged resources and objective metrics.
 */
OptimizationResult run_dual_decomposition_solver(
    const std::vector<std::tuple<size_t, size_t, double>>& edge_list,
    int num_nodes,
    const std::vector<double>& max_node_powers,
    double total_bandwidth_mhz,
    const std::vector<std::tuple<size_t, size_t, double>>& traffic_demands,
    const std::vector<std::map<Link, double>>& flow_fractions,
    double noise_power_dbm_mhz,
    double alpha_tradeoff,
    double tolerance_epsilon
);

/**
 * @brief Executes the baseline Equal Resource Allocation (ERA) strategy.
 * 
 * @param edge_list Tuple list defining the physical topology.
 * @param traffic_demands Tuple list defining commodity traffic flows.
 * @param flow_fractions Routing distributions evaluated for static allocation.
 * @param total_bandwidth_spectrum System-wide spectrum budget.
 * @param max_power_per_node Uniform maximum available transmission power per node.
 * @param noise_power_dbm_per_mhz Noise power spectral density.
 * @return OptimizationResult Structure containing uniformly distributed resources.
 */
OptimizationResult run_static_equal_allocation(
    const std::vector<std::tuple<size_t, size_t, double>>& edge_list, 
    const std::vector<std::tuple<size_t, size_t, double>>& traffic_demands,      
    const std::vector<std::map<Link, double>>& flow_fractions,
    double total_bandwidth_spectrum,
    double max_power_per_node,   
    double noise_power_dbm_per_mhz
);

/**
 * @brief Executes the Game Theory based Greedy baseline for joint routing and resource allocation.
 */
OptimizationResult run_game_greedy_baseline(
    const std::vector<std::tuple<size_t, size_t, double>>& edge_list,
    int num_nodes,
    const std::vector<double>& max_node_powers,
    double total_bandwidth_mhz,
    const std::vector<std::tuple<size_t, size_t, double>>& traffic_demands,
    double noise_power_dbm_mhz
);

/**
 * @brief Executes the Successive Convex Approximation (SCA) baseline algorithm.
 */
OptimizationResult run_sca_baseline(
    const std::vector<std::tuple<size_t, size_t, double>>& edge_list,
    int num_nodes,
    const std::vector<double>& max_node_powers,
    double total_bandwidth_mhz,
    const std::vector<std::tuple<size_t, size_t, double>>& traffic_demands,
    double noise_power_dbm_mhz
);
