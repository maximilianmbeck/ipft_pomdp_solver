#include <iomanip>
#include <solver_ipft/core/solver.hpp>
#include <solver_ipft/solver/ipft.hpp>
#include <solver_ipft/util/output.hpp>
#include <solver_ipft/util/util.hpp>
#include <sstream>
namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                              ToString classes                              */
/* -------------------------------------------------------------------------- */

/* ---------------------- ParticleSetToString class ---------------------- */

ParticleSetToString::ParticleSetToString(std::shared_ptr<POMDP> model)
    : model_(std::move(model)) {}

std::string
ParticleSetToString::shortDescription(const std::vector<State *> &particleSet,
                                      bool terminated) const {
  // Example for short description of ParticleBelief:
  // [PBelief(num_particles_): Mean(getParticleMean())
  // Variance(getParticleVariance()) Terminal(beliefTerminated_)]
  std::stringstream ss;
  State *meanState = State::weightedMean(particleSet, model_);
  State *stdState = State::weightedVariance(particleSet, model_);
  State::varToStd(stdState);
  ss << "[PBelief(" << particleSet.size() << "): Mean(" << *meanState
     << ") Std(" << *stdState << ") Terminal(" << terminated << ")]";
  this->model_->freeState(meanState);
  this->model_->freeState(stdState);
  return ss.str();
}

std::string ParticleSetToString::particleTable(
    const std::vector<State *> &particleSet) const {
  using namespace std; // NOLINT
  // Example for particle table:
  // ----------------------------------
  //  index | weight | value
  // ----------------------------------
  //    0   |   1.0  | 5.500
  // ...
  // constants
  constexpr int smallColumnSize = 8;
  constexpr int largeColumnSize = 20;
  constexpr int numberOfColumns = 3;
  constexpr char fillChar = ' ';
  //
  stringstream ss;
  string lineSep = string(
      2 * smallColumnSize + 1 * largeColumnSize + 2 * numberOfColumns, '-');

  // print header
  ss << lineSep << endl;
  ss << left << setw(smallColumnSize) << setfill(fillChar) << "index"
     << "| ";
  ss << left << setw(smallColumnSize) << setfill(fillChar) << "weight"
     << "| ";
  ss << left << setw(largeColumnSize) << setfill(fillChar) << "value" << endl;
  ss << lineSep << endl;

  // print data
  for (int i = 0; i < particleSet.size(); i++) {
    ss << left << setw(smallColumnSize) << setfill(fillChar) << to_string(i)
       << "| ";
    ss << left << setw(smallColumnSize) << setfill(fillChar)
       << to_string(particleSet[i]->weight_) << "| ";
    ss << left << setw(largeColumnSize) << setfill(fillChar)
       << particleSet[i]->text() << endl;
  }
  ss << lineSep << endl;
  return ss.str();
}

/* -------------------------------------------------------------------------- */
/*                    other print function implementations                    */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                             ToString functions                             */
/* -------------------------------------------------------------------------- */

std::string doubleVecToString(const std::vector<double> &vec) {
  // prints out in format: (x, x, ...)
  std::stringstream ss;
  ss << "(";
  for (auto &num : vec) {
    ss << std::to_string(num) << ", ";
  }
  ss << ")";
  return ss.str();
}

} // namespace solver_ipft
