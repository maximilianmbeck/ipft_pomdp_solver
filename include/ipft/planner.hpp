#pragma once

#include <ipft/plannerbase.hpp>

namespace ipft {

class Planner : public Plannerbase {
protected:
    int step_;
    int round_;

    SimulationStatistics* sim_stats_;

public:
    Planner();

    virtual ~Planner();

    /**
     * @brief Perform one search-execute-update step
     *
     */
    virtual bool runStep();

    virtual void initializePlanner();

    virtual void runPlanningLoop();

    virtual void resetPlanner();

    // Return a pointer to the true sim stats object (ownership stays in planner)
    virtual SimulationStatistics* getSimulationStatsRef() const;

    /**
     * @brief Run and evaluate POMDP planning for a given number of rounds
     *
     */
    virtual int runPlanning(int argc, char* argv[]);

protected:
    /**
     * @brief Loop the search-execute-update process for a given number of steps
     *
     */
    virtual void planningLoop();

    virtual ValuedAction findAction();

    virtual bool takeAction(const ValuedAction& valuedAct);
};

} // namespace ipft