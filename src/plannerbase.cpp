#include "ipft/plannerbase.hpp"
#include <glog/logging.h>
namespace ipft {
Plannerbase::Plannerbase() : model_(nullptr), world_(nullptr), solver_(nullptr) {
    this->rand_ = new Random();
}

Plannerbase::~Plannerbase() {
    if (solver_ != nullptr)
        delete solver_;
    if (world_ != nullptr)
        delete world_;
    if (model_ != nullptr)
        delete model_;
    if (rand_ != nullptr)
        delete rand_;
}

void Plannerbase::displayParameters() const {
    using namespace std;

    std::cout << "Model = " << typeid(*this->model_).name() << endl
              << "Max. search depth = " << Globals::config.search_depth << endl
              << "Search time per step = " << Globals::config.time_per_move << "ms" << endl
              << "Max. simulations count per search = " << Globals::config.max_simulation_count << endl
              << "Number of simulation steps = " << Globals::config.sim_len << endl
              << "Number of solver particles = " << Globals::config.num_solver_particles << endl
              << "Number of search particles = " << Globals::config.num_search_particles << endl
              << "Progressive widening k_obs = " << Globals::config.pw_k_obs << endl
              << "Progressive widening alpha_obs = " << Globals::config.pw_alpha_obs << endl
              << "Minimum particle std = " << Globals::config.min_particle_std << endl
              << "Discount gamma = " << Globals::config.discount_gamma << endl
              << "Information discount = " << Globals::config.inf_discount_gamma << endl
              << "Explore constant c = " << Globals::config.explore_constant_c << endl
              << "Information gathering const. lambda = " << Globals::config.inf_gather_constant_lambda << endl
              << "Record statistics = " << Globals::config.record_statistics << endl
              << endl;
}

void Plannerbase::printPlanningEnd(
    int num_rounds, const std::chrono::time_point<std::chrono::high_resolution_clock>& main_clock_start) const {
    std::cout << std::endl;
    // print completed # num_runs
    std::cout << "Completed " << num_rounds << " run(s)." << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed;
    elapsed = end - main_clock_start;
    std::cout << "Total time: " << elapsed.count() << "ms." << std::endl;
}

} // namespace ipft
