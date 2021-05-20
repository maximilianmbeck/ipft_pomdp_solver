#pragma once

#include <array>
#include <chrono>
#include <solver_ipft/config.hpp>
#include <solver_ipft/core/information_gain.hpp>
#include <solver_ipft/interface/rollout.hpp>
#include <solver_ipft/interface/solver.hpp>

namespace solver_ipft {

class RolloutPolicy;

/**
 * @brief IpftSearchStatistics class
 */
class IpftSearchStatistics : public SearchStatistics {
public:
    double time_search{0.0};                       // ms
    double time_node_selection{0.0};               // ms
    double time_backup{0.0};                       // ms
    double time_belief_update{0.0};                // ms
    double time_information_gain_computation{0.0}; // ms
    double time_node_rollout{0.0};                 // ms
    int num_tree_vnodes{0};
    int num_simulations{0};
    int deepest_simulation_depth{0}; //

    std::vector<int> num_visits_vnodes_on_level;
    std::vector<int> num_vnodes_on_level;
    std::vector<History> actionSequences;

    // convergence eval
    std::vector<double> timesteps;
    std::vector<std::vector<ValuedAction>> valuedActionsPerTimestep;

    explicit IpftSearchStatistics(std::shared_ptr<POMDP> model)
            : SearchStatistics(std::move(model)), num_visits_vnodes_on_level(Globals::config.search_depth),
              num_vnodes_on_level(Globals::config.search_depth + 1){};

    ~IpftSearchStatistics() override = default;

    IpftSearchStatistics(const IpftSearchStatistics& other) = delete;
    IpftSearchStatistics(IpftSearchStatistics&& other) = delete;
    IpftSearchStatistics& operator=(const IpftSearchStatistics&) = delete;
    IpftSearchStatistics& operator=(IpftSearchStatistics&&) = delete;

    friend std::ostream& operator<<(std::ostream& os, const IpftSearchStatistics& statistics);

    std::string text() const override;

protected:
    std::string shortDescription() const;
    std::string printValuedActions() const;
    std::string printActionObservationSequences() const;
    std::string printTimes() const;
    std::string printNodeStats() const;
};


/**
 * @brief Ipft value class
 */
class IpftValue : public Value {
protected:
    static constexpr int componentCount = 2;
    // index 0: stateValue, index 1: informationValue
    std::array<double, componentCount> value_;

public:
    IpftValue();
    IpftValue(const double& stateValue, const double& informationValue);

    ~IpftValue() override = default;
    IpftValue(const IpftValue&) = default;
    IpftValue(IpftValue&&) = default;
    IpftValue& operator=(const IpftValue&) = default;
    IpftValue& operator=(IpftValue&&) = default;

    void add(const Value& val) override;
    void update(const Value& val, int count) override;
    void set(const Value& val) override;
    void setComponent(int index, const double& val) override;
    double getComponent(int index) const override;
    double getWeightedComponent(int index) const override;
    int getComponentCount() const override;
    std::string text() const override;

    double total() const override;
    std::unique_ptr<Value> clone() const override;

    IpftValue& operator+=(const IpftValue& add);
    IpftValue operator*(const double& factor);
    IpftValue operator+(const IpftValue& add);

    friend std::ostream& operator<<(std::ostream& os, const IpftValue& v);
};


/**
 * @brief Class implementing the ipft using the solver interface
 */
class Ipft : public Solver {
protected:
    std::shared_ptr<Random> rand_;
    std::unique_ptr<RolloutPolicy> rolloutPolicy_;
    std::unique_ptr<DiscountedInformationGain> infGainRewardCalculator_;

    mutable std::unique_ptr<IpftSearchStatistics> stats_;

public:
    Ipft(std::shared_ptr<Random> rand,
         std::shared_ptr<POMDP> model,
         std::unique_ptr<Belief>&& belief,
         std::unique_ptr<RolloutPolicy>&& rp);
    Ipft(std::shared_ptr<Random> rand,
         std::shared_ptr<POMDP> model,
         std::unique_ptr<RolloutPolicy>&& rp,
         std::unique_ptr<DiscountedInformationGain> infGainRewardCalc);
    ~Ipft() override = default;

    Ipft(const Ipft&) = delete;
    Ipft(Ipft&&) = delete;
    Ipft& operator=(const Ipft&) = delete;
    Ipft& operator=(Ipft&&) = delete;

    /* ---------------------------- solver interface ----------------------------
     */

    ValuedAction search() override;

    void beliefUpdate(const Action& action, const Observation& obs) override;

    std::unique_ptr<SearchStatistics> getSearchStatistics() override;

    /* ----------------------------- helper methods -----------------------------
     */

    virtual ValuedAction search(double timeout);

    virtual IpftValue simulate(const std::shared_ptr<VNode>& vnode, int depth);

    virtual IpftValue rollout(const std::shared_ptr<VNode>& leaf, int depth);

protected:
    // argmax actions of vnode
    ValuedAction optimalAction(const VNode& vnode) const;

    std::shared_ptr<QNode> ucbActionSelection(const VNode& vnode) const;

    std::shared_ptr<VNode> createVNode(const std::shared_ptr<QNode>& parent,
                                       Observation* obs,
                                       std::unique_ptr<Belief>&& belief,
                                       int level) const;
};


/**
 * @brief Ipft helper function to measure elapsed time.
 */
double stopTime(const std::chrono::time_point<std::chrono::high_resolution_clock>& start);

} // namespace solver_ipft