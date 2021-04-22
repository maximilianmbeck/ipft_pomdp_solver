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
    bool convergence_eval;           // after each episode (call to simulate) time and Q-val stats are recorded
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
          convergence_eval(false),
          print_search_step_results(true) {
    }

    Config(int search_depth_,
           int time_per_move_,
           int max_simulation_count_,
           int num_search_particles_,
           int num_solver_particles_,
           int sim_len_,
           int eval_len_,
           double pw_k_obs_,
           double pw_alpha_obs_,
           double min_particle_std_,
           double discount_gamma_,
           double inf_discount_gamma_,
           double explore_constant_c_,
           double inf_gather_constant_lambda_,
           bool record_statistics,
           bool convergence_eval,
           bool print_search_step_results)
        : search_depth(search_depth_),
          time_per_move(time_per_move_),                // default 1000
          max_simulation_count(max_simulation_count_),  // default 1000
          num_search_particles(num_search_particles_),  // default 20
          num_solver_particles(num_solver_particles_),  // in sample_trajectory.jl: 10000
          sim_len(sim_len_),
          eval_len(eval_len_),
          pw_k_obs(pw_k_obs_),          // default 5.0
          pw_alpha_obs(pw_alpha_obs_),  // default 1/20
          min_particle_std(min_particle_std_),
          discount_gamma(discount_gamma_),
          inf_discount_gamma(inf_discount_gamma_),                  // 1.0 means undiscounted
          explore_constant_c(explore_constant_c_),                  // default 100.0
          inf_gather_constant_lambda(inf_gather_constant_lambda_),  // default 60
          record_statistics(record_statistics),
          convergence_eval(convergence_eval),
          print_search_step_results(print_search_step_results) {
    }
};

}  // namespace solver_ipft