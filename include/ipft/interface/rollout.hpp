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

/* ------------------------- Default rollout policy ------------------------- */

// Belief based rollout with information reward computation
class BeliefInformationPolicy : public RolloutPolicy {
   public:
    BeliefInformationPolicy(const POMDP *model, const Random *rand)
        : RolloutPolicy(model, new RandomActionChooser(rand)) {
        this->infGainRewardCalculator_ = new EntropyInfGain();  // default information reward calculator
    }
    BeliefInformationPolicy(const POMDP *model, const ActionChooser *actionChooser)
        : RolloutPolicy(model, actionChooser) {
        this->infGainRewardCalculator_ = new EntropyInfGain();  // default information reward calculator
    }
    virtual ~BeliefInformationPolicy() {
        delete this->infGainRewardCalculator_;
    }

    virtual IpftValue rollout(Belief *belief, int depth) const;
};

}  // namespace solver_ipft