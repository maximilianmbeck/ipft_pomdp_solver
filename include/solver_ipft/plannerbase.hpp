#pragma once

#include <chrono>
#include <memory>
#include <solver_ipft/core/solver.hpp>
#include <solver_ipft/interface/pomdp.hpp>
#include <solver_ipft/interface/world.hpp>
#include <solver_ipft/simulation_statistics.hpp>
#include <solver_ipft/util/random.hpp>

namespace solver_ipft {
class Plannerbase {
protected:
  std::shared_ptr<Random> rand_;
  std::shared_ptr<POMDP> model_;
  std::shared_ptr<World> world_;
  std::shared_ptr<Solver> solver_;

public:
  Plannerbase();

  virtual ~Plannerbase() = default;

  Plannerbase(const Plannerbase &) = delete;
  Plannerbase(Plannerbase &&) = delete;
  Plannerbase &operator=(const Plannerbase &) = delete;
  Plannerbase &operator=(Plannerbase &&) = delete;

  /**
   * [Essential]
   * @brief Create, initialize, and return a POMDP model
   *
   * @return the created POMDP model
   */
  virtual std::unique_ptr<POMDP> initializeModel() = 0;

  /**
   * [Essential]
   * @brief Create, initialize, and return the world
   *
   * @param model the POMDP model for the world
   * @return the created world
   */
  virtual std::unique_ptr<World>
  initializePOMDPWorld(std::shared_ptr<POMDP> model) = 0;

  /**
   * @brief Initialize the solver
   *
   * @param model
   * @param belief
   * @return Solver*
   */
  virtual std::unique_ptr<Solver>
  initializeSolver(std::shared_ptr<POMDP> model,
                   std::unique_ptr<Belief> &&belief) = 0;

  /**
   * @brief Display global parameters
   *
   */
  virtual void displayParameters() const;

  virtual void printPlanningEnd(
      int num_rounds,
      const std::chrono::time_point<std::chrono::high_resolution_clock>
          &main_clock_start) const;
};

} // namespace solver_ipft