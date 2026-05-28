#ifndef OPTIMIZATION_IPM_SOLVER_H_
#define OPTIMIZATION_IPM_SOLVER_H_

#include <cstddef>
#include <map>
#include <tuple>
#include <vector>

#include "graph.h"

// ==========================================
// Configuration Structures
// ==========================================

/**
 * @brief Configuration parameters for the Interior Point Method (IPM) solver.
 */
struct IPMConfig {
    int max_iterations = 100;
    double convergence_tolerance = 1e-6;
    double inexact_duality_gap_tolerance = 1e-4;
    double lse_smoothing_parameter = 1.0; 

    // Dynamic Centering (Sigma) Configuration
    bool enable_dynamic_centering = true;
    double initial_centering_parameter = 0.5;
    double min_centering_parameter = 1e-3;
    double max_centering_parameter = 0.9;
};

// ==========================================
// Solver API
// ==========================================

/**
 * @brief Solves the multi-commodity network flow routing problem using an Inexact Interior Point Method (IPM).
 * * @param graph The underlying directed network topology.
 * @param network_edges The complete list of available edges with delay/weight data.
 * @param num_nodes The total number of nodes in the graph network.
 * @param commodity_demands The traffic demand matrix specifying source, destination, and volume.
 * @param initial_rate_capacity The baseline rate capacities for all active links.
 * @param initial_power_allocation The baseline power allocations across the network.
 * @param flow_routing_matrix [out] The resulting optimal flow allocation mapping per commodity.
 * @param config The hyperparameter struct governing IPM convergence and centering behavior.
 * @param effective_alpha The fairness scaling parameter adjusting user utility bounds.
 * * @return void
 * * @note The algorithm solves the Log-Sum-Exp (LSE) smoothed dual problem to bypass non-differentiability. 
 * The dynamic centering parameter $\sigma$ adaptively shrinks the duality gap. 
 * The time complexity per IPM iteration is bottlenecked by the Newton system (KKT matrix) inversion, 
 * which scales as $\mathcal{O}(|E|^3)$ in the worst case, where $|E|$ is the number of active decision variables (links).
 */
void solve_ipm_routing(
    const Graph& graph,
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& network_edges,
    int num_nodes,
    const std::vector<std::tuple<std::size_t, std::size_t, double>>& commodity_demands,
    const LinkMap& initial_rate_capacity,
    const LinkMap& initial_power_allocation,
    std::vector<std::map<Link, double>>& flow_routing_matrix,
    IPMConfig config,
    double effective_alpha
);

#endif // OPTIMIZATION_IPM_SOLVER_H_
