#pragma once

#include <solver_ipft/interface/pomdp.hpp>
#include <solver_ipft/interface/rollout.hpp>
#include <solver_ipft/interface/spaces.hpp>
#include <solver_ipft/util/random.hpp>

/* -------------------------------------------------------------------------- */
/*                          Driving POMDP Definition                          */
/* -------------------------------------------------------------------------- */

namespace driving {
using namespace solver_ipft;
/* -------------------------------------------------------------------------- */
/*                       Defintions of Driving constants                      */
/* -------------------------------------------------------------------------- */

constexpr int drivingPrec = 4;  // precision of state and observation outputs / prints

/* -------------------------------------------------------------------------- */
/*             Definitions of state, observation and action space             */
/* -------------------------------------------------------------------------- */

/* ------------------------------- state space ------------------------------ */

class DrivingState : public State {
   public:
    DrivingState();
    // TODO another state constructor

    virtual ~DrivingState();

    bool equals(const Point &p) const override;

    int dimensions() const override;

    void reset() override;

    std::string text() const override;

    double get(int dim = 0) const override;

    void set(const double &p, int dim = 0) override;

    friend std::ostream &operator<<(std::ostream &os, const DrivingState &p);

   private:
    // TODO state space definition here

    double epsilon() const override;
};

/* ---------------------------- observation space --------------------------- */

class DrivingObs : public Observation {
   public:
    DrivingObs();
    // TODO another obs constructor

    virtual ~DrivingObs();

    bool equals(const Point &p) const override;

    int dimensions() const override;

    void reset() override;

    std::string text() const override;

    double get(int dim = 0) const override;

    void set(const double &p, int dim = 0) override;

    friend std::ostream &operator<<(std::ostream &os, const DrivingObs &p);

   private:
    // TODO observation space definition here

    double epsilon() const override;
};

/* ------------------------------ action space ------------------------------ */

// discrete actions
enum class DrivingAction {
    // TODO action space definition here
};

// action value class for printing (for conversion from action index to action value)
class DrivingActionValue : public Point {
    // TODO action index to action value conversion here
};

/* -------------------------------------------------------------------------- */
/*                         Definition of Driving POMDP                        */
/* -------------------------------------------------------------------------- */

// TODO template class with state and obs types?
class Driving : public POMDP {
   protected:
    mutable MemoryPool<DrivingState> state_memory_pool_;
    mutable MemoryPool<DrivingObs> obs_memory_pool_;
    const Random *rand_;

   public:
    /* ----------------------------- Model functions ---------------------------- */
    State *createStartState() const override;  // TODO from simulation
    Belief *initialBelief(std::string type = "DEFAULT") const override;

    State *transition(const State &state, const Action &action) const override;
    Observation *observation(const State &statePosterior) const override;
    double obsProb(const State &statePosterior, const Observation &obs) const override;
    double maxPossibleWeight(const Action &act, const Observation &obs) const override;
    double reward(const State &state, const Action &action) const override;
    bool terminalState(const State &statePosterior, const Action &action) const override;

    int numActions() const override;
    std::unique_ptr<ActionValue> valueOfAction(const Action &act) const override;

    int numDimStateSpace() const override;

    /* ---------------------------- Display function ---------------------------- */
    std::string to_string(const State &state) const override;
    std::string to_string(const Observation &obs) const override;
    std::string to_string(const Action &action) const override;  // TODO consider using CLDActionValue
    std::string to_string(const Belief &belief) const override;

    /* ---------------------------- Memory management --------------------------- */
    Observation *allocateObs() const override;
    State *allocateState() const override;
    Observation *copyObs(const Observation *obs) const override;
    State *copyState(const State *state) const override;
    void freeObs(Observation *obs) const override;
    void freeState(State *state) const override;
    int numActiveObs() const override;
    int numActiveStates() const override;
};

}  // namespace driving
