#pragma once

#include <string>

#include "solver_ipft/core/particle_belief.hpp"

namespace solver_ipft {
// Forward declarations
class ParticleBelief;

/* -------------------------------------------------------------------------- */
/*                              ToString classes                              */
/* -------------------------------------------------------------------------- */

/* ------------------------- ParticleSetToString ------------------------- */

// create a ParticleSetToString class (which is a friend of ParticleBelief)
// does all the conversion to string
// this class is used for better extendability (make this class an interface and use different implementations)
class ParticleSetToString {
   private:
    const POMDP* model_;

   public:
    ParticleSetToString(const POMDP* model);

    std::string shortDescription(const std::vector<State*>& particleSet, bool terminated) const;

    std::string particleTable(const std::vector<State*>& particleSet) const;
};

/* -------------------------------------------------------------------------- */
/*                             ToString functions                             */
/* -------------------------------------------------------------------------- */

std::string doubleVecToString(const std::vector<double>& vec);

}  // namespace solver_ipft