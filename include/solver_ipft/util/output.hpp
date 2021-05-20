#pragma once

#include <memory>
#include <string>

#include <solver_ipft/core/particle_belief.hpp>
namespace solver_ipft {
// Forward declarations
class ParticleBelief;

/* -------------------------------------------------------------------------- */
/*                              ToString classes                              */
/* -------------------------------------------------------------------------- */

/* ------------------------- ParticleSetToString ------------------------- */

/**
 * @brief Handles conversions to string
 *
 */
class ParticleSetToString {
private:
    std::shared_ptr<POMDP> model_;

public:
    explicit ParticleSetToString(std::shared_ptr<POMDP> model);

    std::string shortDescription(const std::vector<State*>& particleSet, bool terminated) const;

    std::string particleTable(const std::vector<State*>& particleSet) const;
};

/* -------------------------------------------------------------------------- */
/*                             ToString functions                             */
/* -------------------------------------------------------------------------- */

std::string doubleVecToString(const std::vector<double>& vec);

} // namespace solver_ipft