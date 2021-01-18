#pragma once

#include <chrono>
#include <ipft/config.hpp>
#include <ipft/core/information_gain.hpp>
#include <ipft/core/solver.hpp>
#include <ipft/interface/rollout.hpp>

namespace solver_ipft {
class RolloutPolicy;

/* -------------------------------------------------------------------------- */
/*                         IpftSearchStatistics class                         */
/* -------------------------------------------------------------------------- */

class IpftSearchStatistics : public SearchStatistics {
   public:
    double time_search;                        // ms
    double time_node_selection;                // ms
    double time_backup;                        // ms
    double time_belief_update;                 // ms
    double time_information_gain_computation;  // ms
    double time_node_rollout;                  // ms
    int num_tree_vnodes;
    int num_simulations;
    int deepest_simulation_depth;  //

    std::vector<int> num_visits_vnodes_on_level;
    std::vector<int> num_vnodes_on_level;
    // ParticleBelief* root_belief;
    std::vector<History> actionSequences;

    IpftSearchStatistics()
        : time_search(0.0),
          time_node_selection(0.0),
          time_backup(0.0),
          time_belief_update(0.0),
          time_information_gain_computation(0.0),
          time_node_rollout(0.0),
          num_tree_vnodes(0),
          num_simulations(0),
          deepest_simulation_depth(0),
          num_visits_vnodes_on_level(Globals::config.search_depth),
          num_vnodes_on_level(Globals::config.search_depth + 1)
          //   root_belief(nullptr)
          {
              // num_visits_vnodes_on_level.resize(Globals::config.search_depth);
              // num_vnodes_on_level.resize(Globals::config.search_depth);
          };

    virtual ~IpftSearchStatistics();

    // IpftSearchStatistics(const IpftSearchStatistics &other);

    // IpftSearchStatistics(IpftSearchStatistics &&other) = delete;

    friend std::ostream& operator<<(std::ostream& os, const IpftSearchStatistics& statistics);

    std::string text() const override;

   protected:
    std::string shortDescription() const;
    std::string printValuedActions() const;
    std::string printActionObservationSequences() const;
    std::string printTimes() const;
    std::string printNodeStats() const;
};

/* -------------------------------------------------------------------------- */
/*                              Ipft value class                              */
/* -------------------------------------------------------------------------- */

class IpftValue : public Value {
   protected:
    static constexpr int componentCount = 2;
    // index 0: stateValue, index 1: informationValue
    double value[componentCount];

   public:
    IpftValue();
    IpftValue(const double& stateValue, const double& informationValue);
    virtual ~IpftValue() {
    }

    virtual void add(const Value& val) override;
    virtual void update(const Value& val, int count) override;
    virtual void set(const Value& val) override;
    virtual void setComponent(int index, const double& val) override;
    virtual double getRawComponent(int index) const override;
    virtual double getWeightedComponent(int index) const override;
    virtual int getComponentCount() const override;
    virtual std::string text() const override;

    virtual double total() const override;
    virtual Value* clone() const override;

    IpftValue& operator+=(const IpftValue& add);
    IpftValue operator*(const double& factor);
    IpftValue operator+(const IpftValue& add);

    friend std::ostream& operator<<(std::ostream& os, const IpftValue& v);
};

/* -------------------------------------------------------------------------- */
/*                              Ipft solver class                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Class implementing the ipft using the solver interface
 *
 */
class Ipft : public Solver {
   protected:
    const Random* rand_;
    RolloutPolicy* rolloutPolicy_;
    DiscountedInformationGain* infGainRewardCalculator_;

    mutable IpftSearchStatistics* stats_;

   public:
    Ipft(const POMDP* model, Belief* belief, const Random* rand, RolloutPolicy* rp);
    Ipft(const POMDP* model, const Random* rand, RolloutPolicy* rp, DiscountedInformationGain* infGainRewardCalc);
    virtual ~Ipft();

    /* ---------------------------- solver interface ---------------------------- */

    virtual ValuedAction search();

    virtual void beliefUpdate(const Action& action, const Observation& obs);

    virtual void setBelief(Belief* b);

    virtual Belief* getBelief() const;

    virtual SearchStatistics* getSearchStatistics() const;

    /* ----------------------------- helper methods ----------------------------- */

    virtual ValuedAction search(double timeout);

    virtual IpftValue simulate(VNode* vnode, int depth);

    virtual IpftValue rollout(VNode* leaf, int depth);

   protected:
    // argmax actions of vnode
    ValuedAction optimalAction(const VNode* vnode) const;

    // QNode * actionProgressiveWidening(VNode * vnode) const;

    QNode* ucbActionSelection(VNode* vnode) const;

    VNode* createVNode(QNode* parent, Observation* obs, Belief* belief, int level) const;
};

/* -------------------------------------------------------------------------- */
/*                            Ipft helper functions                           */
/* -------------------------------------------------------------------------- */

double stopTime(const std::chrono::time_point<std::chrono::high_resolution_clock>& start);

}  // namespace solver_ipft