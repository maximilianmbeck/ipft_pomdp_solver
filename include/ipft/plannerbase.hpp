#pragma once

#include <ipft/simulation_statistics.hpp>
#include <ipft/core/solver.hpp>
#include <ipft/interface/pomdp.hpp>
#include <ipft/interface/world.hpp>

#include <ipft/util/random.hpp>

#include <chrono>

namespace solver_ipft {
class Plannerbase {
protected:
    Random* rand_;
    POMDP* model_;
    World* world_;
    Solver* solver_;

public:
    Plannerbase();

    virtual ~Plannerbase();

    /**
     * [Essential]
     * @brief Create, initialize, and return a POMDP model
     *
     * @return POMDP* the created POMDP model
     */
    virtual POMDP* initializeModel() = 0;

    /**
     * [Essential]
     * @brief Create, initialize, and return the world
     *
     * @param model the POMDP model for the world
     * @return World* the created world
     */
    virtual World* initializePOMDPWorld(POMDP* model) = 0;

    /**
     * @brief Initialize the solver
     *
     * @param model
     * @param belief
     * @return Solver*
     */
    virtual Solver* initializeSolver(POMDP* model, Belief* belief) = 0;

    /**
     * @brief Display global parameters
     *
     */
    virtual void displayParameters() const;

    virtual void printPlanningEnd(int num_rounds,
                                  const std::chrono::time_point<std::chrono::high_resolution_clock>& main_clock_start) const;
};

} // namespace solver_ipft