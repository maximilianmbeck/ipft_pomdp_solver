#pragma once

namespace solver_ipft {

struct Config {
    // search parameters
    int search_depth;
    int time_per_move;  // in milliseconds, time available to construct the search tree
    int max_simulation_count;
    int num_search_particles;  // number of particle used for the beliefs during treesearch
    int num_solver_particles;  // number of particles of the solver to track the real world state

    // world simulation/evaluation parameters
    int sim_len;   // number of steps to run the simulation for
    int eval_len;  // number of rounds to evaluate the solver

    // progressive widening parameters
    // * no progressive widening on action space
    // double pw_k_action;
    // double pw_alpha_action;
    // observations
    double pw_k_obs;
    double pw_alpha_obs;

    // search parameters
    double min_particle_std;

    // reward calculation parameters
    double discount_gamma;
    double explore_constant_c;          // ucb explore constant c
    double inf_gather_constant_lambda;  // weights reward maximization and information gathering
    double inf_discount_gamma;          // = 1.0 for undiscounted information gain

    // record statistics
    bool record_statistics;
    bool print_search_step_results;  // verbose output of each search tree

    Config()
        : search_depth(20), time_per_move(1000),  // default 1000
          max_simulation_count(100000),           // default 1000
          num_search_particles(20),               // default 20
          num_solver_particles(10000),            // in sample_trajectory.jl: 10000
          sim_len(40),
          eval_len(100),
          pw_k_obs(5.0),       // default 5.0
          pw_alpha_obs(0.05),  // default 1/20
          min_particle_std(0.1),
          discount_gamma(0.95),
          inf_discount_gamma(1.0),           // 1.0 means undiscounted
          explore_constant_c(100.0),         // default 100.0
          inf_gather_constant_lambda(60.0),  // default 60
          record_statistics(true),
          print_search_step_results(true) {
    }
};

}  // namespace solver_ipft