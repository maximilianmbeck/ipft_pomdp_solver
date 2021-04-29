#pragma once

#include <solver_ipft/core/information_gain.hpp>
#include <solver_ipft/core/node.hpp>
#include <solver_ipft/interface/pomdp.hpp>
#include <solver_ipft/solver/ipft.hpp>

namespace solver_ipft {
class IpftValue;

/* -------------------------------------------------------------------------- */
/*                                 Interfaces                                 */
/* -------------------------------------------------------------------------- */

/* --------------------- ActionChooser interface --------------------- */

// implement this interface to incorporate domain knowledge
class ActionChooser {
public:
  ActionChooser() = default;
  virtual ~ActionChooser() = default;

  ActionChooser(const ActionChooser &) = delete;
  ActionChooser(ActionChooser &&) = delete;
  ActionChooser &operator=(const ActionChooser &) = delete;
  ActionChooser &operator=(ActionChooser &&) = delete;

  virtual Action chooseAction(const Belief *belief) const = 0;
};

/* ---------------------------- Rollout interface --------------------------- */

// Interface for the rollout policy
class RolloutPolicy {
protected:
  const POMDP *model_;
  const ActionChooser *actionChooser_;
  const DiscountedInformationGain *infGainRewardCalculator_;

public:
  RolloutPolicy(const POMDP *model, const ActionChooser *actionChooser)
      : model_(model), actionChooser_(actionChooser),
        infGainRewardCalculator_(nullptr) {}
  RolloutPolicy(const POMDP *model, const ActionChooser *actionChooser,
                const DiscountedInformationGain *infGainRewardCalc)
      : model_(model), actionChooser_(actionChooser),
        infGainRewardCalculator_(infGainRewardCalc) {}
  virtual ~RolloutPolicy() {
    delete actionChooser_;
    delete infGainRewardCalculator_;
  }

  RolloutPolicy(const RolloutPolicy &) = delete;
  RolloutPolicy(RolloutPolicy &&) = delete;
  RolloutPolicy &operator=(const RolloutPolicy &) = delete;
  RolloutPolicy &operator=(RolloutPolicy &&) = delete;

  virtual IpftValue rollout(Belief *belief, int depth) const = 0;
};

/* -------------------------------------------------------------------------- */
/*                     Classes implementing the interface                     */
/* -------------------------------------------------------------------------- */

/* ------------------------- Random action selection ------------------------ */
class RandomActionChooser : public ActionChooser {
protected:
  const Random *rand_;

public:
  explicit RandomActionChooser(const Random *rand) : rand_(rand) {}
  ~RandomActionChooser() override = default;

  RandomActionChooser(const RandomActionChooser &) = delete;
  RandomActionChooser(RandomActionChooser &&) = delete;
  RandomActionChooser &operator=(const RandomActionChooser &) = delete;
  RandomActionChooser &operator=(RandomActionChooser &&) = delete;

  Action chooseAction(const Belief *belief) const override;
};

/* --------------------- Deterministic action selection --------------------- */

class DeterministicActionChooser : public ActionChooser {
protected:
  std::vector<Action> actions_;
  mutable int round_;

public:
  explicit DeterministicActionChooser(const std::vector<Action> &actions);

  ~DeterministicActionChooser() override = default;

  DeterministicActionChooser(const DeterministicActionChooser &) = delete;
  DeterministicActionChooser(DeterministicActionChooser &&) = delete;
  DeterministicActionChooser &
  operator=(const DeterministicActionChooser &) = delete;
  DeterministicActionChooser &operator=(DeterministicActionChooser &&) = delete;

  Action chooseAction(const Belief *belief) const override;

  int numberOfActionsLeft() const;

  void reset();
};

class DeterministicSingleActionChooser : public ActionChooser {
protected:
  Action deterministicAction;

public:
  explicit DeterministicSingleActionChooser(const Action &act)
      : deterministicAction(act) {}
  ~DeterministicSingleActionChooser() override = default;

  DeterministicSingleActionChooser(const DeterministicSingleActionChooser &) =
      delete;
  DeterministicSingleActionChooser(DeterministicSingleActionChooser &&) =
      delete;
  DeterministicSingleActionChooser &
  operator=(const DeterministicSingleActionChooser &) = delete;
  DeterministicSingleActionChooser &
  operator=(DeterministicSingleActionChooser &&) = delete;

  Action chooseAction(const Belief *belief) const override;
};

/* --------------------------- BeliefRolloutPolicy -------------------------- */

// Belief based rollout
class BeliefRolloutPolicy : public RolloutPolicy {
public:
  BeliefRolloutPolicy(const POMDP *model, const ActionChooser *actionChooser,
                      const DiscountedInformationGain *infGainRewardCalculator)
      : RolloutPolicy(model, actionChooser, infGainRewardCalculator) {}

  ~BeliefRolloutPolicy() override = default;

  BeliefRolloutPolicy(const BeliefRolloutPolicy &) = delete;
  BeliefRolloutPolicy(BeliefRolloutPolicy &&) = delete;
  BeliefRolloutPolicy &operator=(const BeliefRolloutPolicy &) = delete;
  BeliefRolloutPolicy &operator=(BeliefRolloutPolicy &&) = delete;

  IpftValue rollout(Belief *belief, int depth) const override;
};

/* ------------------------- Default rollout policy ------------------------- */

// Belief based rollout with information reward computation
class BeliefInformationPolicy : public BeliefRolloutPolicy {
public:
  BeliefInformationPolicy(const POMDP *model, const Random *rand)
      : BeliefRolloutPolicy(model, new RandomActionChooser(rand),
                            new EntropyInfGain()) {}
  BeliefInformationPolicy(const POMDP *model,
                          const ActionChooser *actionChooser)
      : BeliefRolloutPolicy(model, actionChooser, new EntropyInfGain()) {}

  ~BeliefInformationPolicy() override = default;
  BeliefInformationPolicy(const BeliefInformationPolicy &) = delete;
  BeliefInformationPolicy(BeliefInformationPolicy &&) = delete;
  BeliefInformationPolicy &operator=(const BeliefInformationPolicy &) = delete;
  BeliefInformationPolicy &operator=(BeliefInformationPolicy &&) = delete;
};

} // namespace solver_ipft