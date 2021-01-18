#pragma once

#include "ipft/core/information_gain.hpp"
#include "ipft/core/node.hpp"
#include "ipft/interface/pomdp.hpp"
#include "ipft/solver/ipft.hpp"

namespace solver_ipft {
class IpftValue;

/* -------------------------------------------------------------------------- */
/*                                 Interfaces                                 */
/* -------------------------------------------------------------------------- */

/* --------------------- ActionChooser interface --------------------- */

// implement this interface to incorporate domain knowledge
class ActionChooser {
   public:
    virtual ~ActionChooser() {
    }
    virtual Action chooseAction(const Belief *belief) const = 0;
};

/* ---------------------------- Rollout interface --------------------------- */
class RolloutPolicy  // TODO: make abstraction for general rollout policy for all solvers
{
   protected:
    const POMDP *model_;
    const ActionChooser *actionChooser_;
    const DiscountedInformationGain *infGainRewardCalculator_;

   public:
    RolloutPolicy(const POMDP *model, const ActionChooser *actionChooser)
        : model_(model), actionChooser_(actionChooser), infGainRewardCalculator_(nullptr) {
    }
    RolloutPolicy(const POMDP *model,
                  const ActionChooser *actionChooser,
                  const DiscountedInformationGain *infGainRewardCalc)
        : model_(model), actionChooser_(actionChooser), infGainRewardCalculator_(infGainRewardCalc) {
    }
    virtual ~RolloutPolicy() {
        delete actionChooser_;
        if (infGainRewardCalculator_ != nullptr) {
            delete infGainRewardCalculator_;
        }
    }

    virtual IpftValue rollout(Belief *belief,
                              int depth) const = 0;  // TODO more abstract: return value instead of IpftValue
};

/* -------------------------------------------------------------------------- */
/*                     Classes implementing the interface                     */
/* -------------------------------------------------------------------------- */

/* ------------------------- Random action selection ------------------------ */

class RandomActionChooser : public ActionChooser {
   protected:
    const Random *rand_;

   public:
    RandomActionChooser(const Random *rand) : rand_(rand) {
    }
    virtual ~RandomActionChooser() {
    }
    Action chooseAction(const Belief *belief) const override;
};

/* --------------------- Deterministic action selection --------------------- */

class DeterministicActionChooser : public ActionChooser {
   protected:
    std::vector<Action> actions_;
    mutable int round_;

   public:
    DeterministicActionChooser(std::vector<Action> actions);

    virtual ~DeterministicActionChooser();

    virtual Action chooseAction(const Belief *belief) const override;

    int numberOfActionsLeft() const;

    void reset();
};

class DeterministicSingleActionChooser : public ActionChooser {
   protected:
    Action deterministicAction;

   public:
    DeterministicSingleActionChooser(const Action &act) : deterministicAction(act) {}
    virtual ~DeterministicSingleActionChooser() {}
    virtual Action chooseAction(const Belief *belief) const override;
};

/* --------------------------- BeliefRolloutPolicy -------------------------- */

// Belief based rollout
class BeliefRolloutPolicy : public RolloutPolicy {
   public:
    BeliefRolloutPolicy(const POMDP *model, const ActionChooser *actionChooser, const DiscountedInformationGain *infGainRewardCalculator)
        : RolloutPolicy(model, actionChooser, infGainRewardCalculator) {}

    virtual ~BeliefRolloutPolicy() {}

    virtual IpftValue rollout(Belief *belief, int depth) const;
};

/* ------------------------- Default rollout policy ------------------------- */

// Belief based rollout with information reward computation
class BeliefInformationPolicy : public BeliefRolloutPolicy {
   public:
    BeliefInformationPolicy(const POMDP *model, const Random *rand)
        : BeliefRolloutPolicy(model, new RandomActionChooser(rand), new EntropyInfGain()) {
    }
    BeliefInformationPolicy(const POMDP *model, const ActionChooser *actionChooser)
        : BeliefRolloutPolicy(model, actionChooser, new EntropyInfGain()) {
    }
    virtual ~BeliefInformationPolicy() {}

    // virtual IpftValue rollout(Belief *belief, int depth) const;
};

}  // namespace solver_ipft