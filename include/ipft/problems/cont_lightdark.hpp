#pragma once

/* -------------------------------------------------------------------------- */
/*                   Continuous Light Dark POMDP definition                   */
/* -------------------------------------------------------------------------- */

#include <ipft/interface/pomdp.hpp>
#include <ipft/interface/rollout.hpp>
#include <ipft/interface/spaces.hpp>
#include <ipft/util/random.hpp>

namespace solver_ipft {

namespace cld {

/* -------------------------------------------------------------------------- */
/*                         Definitions of CLD constants                       */
/* -------------------------------------------------------------------------- */

constexpr double epsilonCLDState = 1e-6;
constexpr double epsilonCLDObs = 1e-6;
constexpr int numberOfActions = 5;
constexpr double muInitial = 0.0;      // see MA Fischer p.87 + continuous_ld.jl
constexpr double sigmaInitial = 10.0;  // see MA Fischer p.87 + continuous_ld.jl
constexpr double sigmaTransition = 0.1;
constexpr double lightSourceLoc = 10.0;
constexpr double goalRegion = 0.0;
constexpr int actToValueMap[5] = {-3, -1, 0, 1, 3};  // TODO replace any direct access to this array by CLDActionValue class

constexpr int cldPrec = 4;  // precision of state and observation outputs / prints

/* -------------------------------------------------------------------------- */
/*             Definitions of state, observation and action space             */
/* -------------------------------------------------------------------------- */

// continuous state space
class CLDState : public State {
   public:
    CLDState() : posState(0){};
    CLDState(double state) : posState(state){};
    virtual ~CLDState() = default;

    bool equals(const Point &p) const override;

    // double distanceTo(const Point &p) const override;

    int dimensions() const override;

    void reset() override;

    // bool operator<(const Point &rhs) const override;

    // bool operator==(const Point &rhs) const override;

    std::string text() const override;

    double get(int dim = 0) const override;

    void set(const double &p, int dim = 0) override;

    friend std::ostream &operator<<(std::ostream &os, const CLDState &p);

   private:
    double posState;

    double epsilon() const override;
};

// continuous observation space
class CLDObs : public Observation {
   public:
    CLDObs() : posObs(0){};
    CLDObs(double obs) : posObs(obs){};
    virtual ~CLDObs() = default;

    bool equals(const Point &p) const override;

    // double distanceTo(const Point &p) const override;

    int dimensions() const override;

    void reset() override;

    // bool operator<(const Point &rhs) const override;

    // bool operator==(const Point &rhs) const override;

    std::string text() const override;

    double get(int dim = 0) const override;

    void set(const double &p, int dim = 0) override;

    friend std::ostream &operator<<(std::ostream &os, const CLDObs &p);

   private:
    double posObs;

    double epsilon() const override;
};

// Scoped declaration of discrete action space
enum class CLDAction {
    NEG3 = 0,
    NEG1 = 1,
    ZERO = 2,
    POS1 = 3,
    POS3 = 4
};

// action value class
class CLDActionValue : public CLDObs {
   public:
    CLDActionValue(const Action &act);
    virtual ~CLDActionValue() = default;

    friend std::ostream &operator<<(std::ostream &os, const CLDActionValue &p);
};

/* -------------------------------------------------------------------------- */
/*                           Definition of the POMDP                          */
/* -------------------------------------------------------------------------- */

class ContLightDark : public POMDP {
   protected:
    mutable MemoryPool<CLDState> state_memory_pool_;
    mutable MemoryPool<CLDObs> obs_memory_pool_;
    const Random *rand_;

   public:
    ContLightDark(const Random *rand) : rand_(rand){};
    virtual ~ContLightDark();

    /* ----------------------- Simulative model functions ----------------------- */
    State *createStartState() const override;
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
    std::vector<State *> similarStates(const State &state, int count) const override;
    void newParticle(State *particle, const std::vector<State *> &particleSet, const Action &act, const Observation &obs) const override;

    // only for this model
    double obsModelSigma(const double &statePos) const;

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

}  // namespace cld
}  // namespace solver_ipft