#pragma once
#include <pybind11/pybind11.h>

#include <ipft/solver/ipft.hpp>

#include "py_node.hpp"
#include "py_particle_belief.hpp"

namespace ipft {
namespace pyadapters {

/**
 * @brief Wrapper for Ipft class
 * 
 * Use this to execute single searches and belief updates with an ipft solver object
 * 
 */
class PyIpft {
   protected:
    Random* rand_;
    Ipft* ipft_;
    ParticleBelief* initialBelief_;
    POMDP* model_;

   public:
    // pass initial belief
    PyIpft() : initialBelief_(nullptr), ipft_(nullptr) {
        this->rand_ = new Random();
    }

    virtual ~PyIpft() {
        if (this->ipft_ != nullptr)
            delete ipft_;
        delete this->initialBelief_;
        delete this->model_;
        delete this->rand_;
    }

    // override this function to use ipft solver with a different model
    virtual POMDP* initializeModel() const = 0;

    // reset ipft: set back to initial belief
    virtual void reset();

    virtual std::shared_ptr<PyVNode> getSearchTree() const;

    // returns a tuple <Action, Value>
    virtual py::tuple search();  // TODO from here add total value to tuple

    // TODO: simulate run one simulate + expand step
    // virtual py::array_t<double> simulate();

    // TODO: belief update (need conversion from python type to Action and Observation)

    // get belief
    virtual PyParticleBelief getBelief() const;

   protected:
    virtual void initializeSolver();
};

class PyIpftCld : public PyIpft {
   public:
    PyIpftCld(const py::array_t<double>& initialParticleBeliefArray) {
        this->model_ = initializeModel();
        std::vector<State*> particleSet = npArray2ParticleSet(initialParticleBeliefArray, this->model_);
        this->initialBelief_ = new ParticleBelief(particleSet, false, this->model_, this->rand_, new ObsAdaptiveReinvigorator(this->model_, this->rand_));
        initializeSolver();
    }

    virtual POMDP* initializeModel() const override;
};

}  // namespace pyadapters
}  // namespace ipft
