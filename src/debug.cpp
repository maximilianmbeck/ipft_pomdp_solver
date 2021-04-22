#include "solver_ipft/util/debug.hpp"

#include "solver_ipft/core/particle_belief.hpp"

namespace solver_ipft {

namespace debug {

bool allParticlesEqual(const Belief* belief) {
    const ParticleBelief* pBelief = static_cast<const ParticleBelief*>(belief);
    for (int i = 1; i < pBelief->numParticles(); i++) {
        const State* prevState = pBelief->particle(i - 1);
        const State* curState = pBelief->particle(i);
        if (!(prevState->equals(*curState))) {
            return false;
        }
    }
    return true;
}

bool allParticlesEqual(const std::vector<State*>& particles) {
    for (int i = 1; i < particles.size(); i++) {
        const State* prevState = particles[i - 1];
        const State* curState = particles[i];
        if (!(prevState->equals(*curState))) {
            return false;
        }
    }
    return true;
}

std::vector<State*> doubleVec2StateVec(const std::vector<double>& numbers, POMDP* model) {
    std::vector<State*> states;
    // generate 1 dimensional states from number vector
    for (double n : numbers) {
        State* s = model->allocateState();
        s->set(n, 0);
        s->weight_ = 1.0 / numbers.size();
        states.push_back(s);
    }

    return states;
}

void freeStateVec(std::vector<State*> states) {
    for (State* s : states) {
        delete s;
    }
}

void freePointVec(std::vector<Point*> points) {
    for (Point* p : points) {
        delete p;
    }
}

}  // namespace debug

}  // namespace solver_ipft