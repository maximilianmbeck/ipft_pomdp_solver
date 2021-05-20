#pragma once

#include <solver_ipft/core/information_gain.hpp>
#include <solver_ipft/core/node.hpp>
#include <solver_ipft/interface/pomdp.hpp>
#include <solver_ipft/solver/ipft.hpp>

namespace solver_ipft {
class IpftValue;


/**
 * @brief   Interface for action selection strategy in the rollout policy.
 * With this, incorporating domain knowledge during rollout is possible.
 */
class ActionChooser {
public:
    ActionChooser() = default;
    virtual ~ActionChooser() = default;

    ActionChooser(const ActionChooser&) = delete;
    ActionChooser(ActionChooser&&) = delete;
    ActionChooser& operator=(const ActionChooser&) = delete;
    ActionChooser& operator=(ActionChooser&&) = delete;

    virtual Action chooseAction(const Belief* belief) const = 0;
};


/**
 * @brief   Interface for the rollout policy
 * Use design principle "template method pattern".
 */
class RolloutPolicy {
protected:
    const std::shared_ptr<POMDP> model_;
    std::unique_ptr<ActionChooser> actionChooser_;
    std::unique_ptr<DiscountedInformationGain> infGainRewardCalculator_;

public:
    RolloutPolicy(std::shared_ptr<POMDP> model, std::unique_ptr<ActionChooser>&& actionChooser)
            : model_(std::move(model)), actionChooser_(std::move(actionChooser)), infGainRewardCalculator_(nullptr) {
    }
    RolloutPolicy(std::shared_ptr<POMDP> model,
                  std::unique_ptr<ActionChooser>&& actionChooser,
                  std::unique_ptr<DiscountedInformationGain>&& infGainRewardCalc)
            : model_(std::move(model)), actionChooser_(std::move(actionChooser)),
              infGainRewardCalculator_(std::move(infGainRewardCalc)) {
    }
    virtual ~RolloutPolicy() = default;

    RolloutPolicy(const RolloutPolicy&) = delete;
    RolloutPolicy(RolloutPolicy&&) = delete;
    RolloutPolicy& operator=(const RolloutPolicy&) = delete;
    RolloutPolicy& operator=(RolloutPolicy&&) = delete;

    virtual IpftValue rollout(std::unique_ptr<Belief>&& belief, int depth) const = 0;
};

/**
 * @brief   Choose actions randomly from action space.
 */
class RandomActionChooser : public ActionChooser {
protected:
    const std::shared_ptr<Random> rand_;

public:
    explicit RandomActionChooser(std::shared_ptr<Random> rand) : rand_(std::move(rand)) {
    }
    ~RandomActionChooser() override = default;

    RandomActionChooser(const RandomActionChooser&) = delete;
    RandomActionChooser(RandomActionChooser&&) = delete;
    RandomActionChooser& operator=(const RandomActionChooser&) = delete;
    RandomActionChooser& operator=(RandomActionChooser&&) = delete;

    Action chooseAction(const Belief* belief) const override;
};

/**
 * @brief   Chooses predefined actions.
 * Using 'actions' vector, chooses actions based on their indices.
 */
class DeterministicActionChooser : public ActionChooser {
protected:
    std::vector<Action> actions_;
    mutable int round_;

public:
    explicit DeterministicActionChooser(std::vector<Action> actions);

    ~DeterministicActionChooser() override = default;

    DeterministicActionChooser(const DeterministicActionChooser&) = delete;
    DeterministicActionChooser(DeterministicActionChooser&&) = delete;
    DeterministicActionChooser& operator=(const DeterministicActionChooser&) = delete;
    DeterministicActionChooser& operator=(DeterministicActionChooser&&) = delete;

    Action chooseAction(const Belief* belief) const override;

    int numberOfActionsLeft() const;

    void reset();
};

/**
 * @brief   Chooses a predefined action repeatedly.
 */
class DeterministicSingleActionChooser : public ActionChooser {
protected:
    Action deterministicAction;

public:
    explicit DeterministicSingleActionChooser(const Action& act) : deterministicAction(act) {
    }
    ~DeterministicSingleActionChooser() override = default;

    DeterministicSingleActionChooser(const DeterministicSingleActionChooser&) = delete;
    DeterministicSingleActionChooser(DeterministicSingleActionChooser&&) = delete;
    DeterministicSingleActionChooser& operator=(const DeterministicSingleActionChooser&) = delete;
    DeterministicSingleActionChooser& operator=(DeterministicSingleActionChooser&&) = delete;

    Action chooseAction(const Belief* belief) const override;
};


/**
 * @brief   Belief based rollout.
 * Propagates belief for estimating the value.
 */
class BeliefRolloutPolicy : public RolloutPolicy {
public:
    BeliefRolloutPolicy(std::shared_ptr<POMDP> model,
                        std::unique_ptr<ActionChooser>&& actionChooser,
                        std::unique_ptr<DiscountedInformationGain>&& infGainRewardCalculator)
            : RolloutPolicy(std::move(model), std::move(actionChooser), std::move(infGainRewardCalculator)) {
    }

    ~BeliefRolloutPolicy() override = default;

    BeliefRolloutPolicy(const BeliefRolloutPolicy&) = delete;
    BeliefRolloutPolicy(BeliefRolloutPolicy&&) = delete;
    BeliefRolloutPolicy& operator=(const BeliefRolloutPolicy&) = delete;
    BeliefRolloutPolicy& operator=(BeliefRolloutPolicy&&) = delete;

    IpftValue rollout(std::unique_ptr<Belief>&& belief, int depth) const override;
};


/**
 * @brief   Belief based rollout with information reward computation.
 * Propagates belief for estimating the value.
 */
class BeliefInformationPolicy : public BeliefRolloutPolicy {
public:
    BeliefInformationPolicy(std::shared_ptr<POMDP> model, std::shared_ptr<Random> rand)
            : BeliefRolloutPolicy(std::move(model),
                                  std::make_unique<RandomActionChooser>(std::move(rand)),
                                  std::make_unique<EntropyInfGain>()) {
    }
    BeliefInformationPolicy(std::shared_ptr<POMDP> model, std::unique_ptr<ActionChooser>&& actionChooser)
            : BeliefRolloutPolicy(std::move(model), std::move(actionChooser), std::make_unique<EntropyInfGain>()) {
    }

    ~BeliefInformationPolicy() override = default;
    BeliefInformationPolicy(const BeliefInformationPolicy&) = delete;
    BeliefInformationPolicy(BeliefInformationPolicy&&) = delete;
    BeliefInformationPolicy& operator=(const BeliefInformationPolicy&) = delete;
    BeliefInformationPolicy& operator=(BeliefInformationPolicy&&) = delete;
};

} // namespace solver_ipft