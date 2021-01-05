// #include <ipft/problems/driving_intersection.hpp>

// namespace ipft {
// namespace driving {

// /* -------------------------------------------------------------------------- */
// /*                             DrivingState class                             */
// /* -------------------------------------------------------------------------- */

// /* -------------------------------------------------------------------------- */
// /*                              DrivingObs class                              */
// /* -------------------------------------------------------------------------- */

// /* -------------------------------------------------------------------------- */
// /*                                Driving class                               */
// /* -------------------------------------------------------------------------- */

// /* ----------------------------- Model functions ---------------------------- */

// State *Driving::createStartState() const {
//     // probably not needed
//     return nullptr;
// }

// Belief *Driving::initialBelief(std::string type) const {
//     // TODO implement
// }

// int Driving::numActions() const {
//     return 0;  // TODO implement
// }

// int Driving::numDimStateSpace() const {
//     return 0;  // TODO implement
// }

// std::unique_ptr<ActionValue> Driving::valueOfAction(const Action &act) const {
//     // TODO implement
//     return std::make_unique<DrivingActionValue>(act);  // create a CLDActionValue object and return a unique pointer to it
// }

// State *Driving::transition(const State &state, const Action &action) const {
//     return nullptr;  // TODO implement
// }

// Observation *Driving::observation(const State &statePosterior) const {
//     // TODO implement
//     return nullptr;
// }

// double Driving::obsProb(const State &statePosterior, const Observation &obs) const {
//     // TODO implement
//     return 0.0;
// }

// double Driving::maxPossibleWeight(const Action &act, const Observation &obs) const {
//     // TODO implement
//     // CLDState sp(obs.get(0));
//     // return obsProb(sp, obs);
//     return 0.0;
// }

// double Driving::reward(const State &state, const Action &action) const {
//     // TODO implement
//     return 0.0;
// }

// bool Driving::terminalState(const State &statePosterior, const Action &action) const {
//     return false;  // TODO implement
// }

// /* ---------------------------- Display functions --------------------------- */
// std::string Driving::to_string(const State &state) const {
//     // std::stringstream ss;
//     // ss.precision(cldPrec);
//     // const CLDState *cldState = static_cast<const CLDState *>(&state);
//     // if (cldState != nullptr)
//     //     ss << "[" << *cldState << " State]";
//     // else
//     //     ss << "[ NULL State]";
//     // return ss.str();
// }
// std::string Driving::to_string(const Observation &obs) const {
//     // std::stringstream ss;
//     // ss.precision(cldPrec);
//     // const CLDObs *cldObs = static_cast<const CLDObs *>(&obs);
//     // if (cldObs != nullptr)
//     //     ss << "[" << *cldObs << " Obs]";
//     // else
//     //     ss << "[ NULL Obs]";
//     // return ss.str();
// }
// std::string Driving::to_string(const Action &action) const {
//     // std::stringstream ss;
//     // ss.precision(1);
//     // CLDActionValue cldAct(action);
//     // ss << "[" << std::setfill(' ') << std::setw(2) << std::right << cldAct.get(0) << " Act]";
//     // return ss.str();
// }
// std::string Driving::to_string(const Belief &belief) const {
//     return belief.text();
// }
// /* ----------------------- Memory management functions ---------------------- */

// Observation *
// Driving::allocateObs() const {
//     DrivingObs *obs = obs_memory_pool_.Allocate();
//     return obs;
// }

// State *Driving::allocateState() const {
//     DrivingState *state = state_memory_pool_.Allocate();
//     return state;
// }

// Observation *Driving::copyObs(const Observation *obs) const {
//     DrivingObs *newObs = obs_memory_pool_.Allocate();
//     *newObs = *static_cast<const DrivingObs *>(obs);  // default copy constructor called
//     newObs->SetAllocated();
//     return newObs;
// }

// State *Driving::copyState(const State *state) const {
//     DrivingState *newState = state_memory_pool_.Allocate();
//     *newState = *static_cast<const DrivingState *>(state);  // default copy constructor called
//     newState->SetAllocated();
//     return newState;
// }

// void Driving::freeObs(Observation *obs) const {
//     obs_memory_pool_.Free(static_cast<DrivingObs *>(obs));
// }

// void Driving::freeState(State *state) const {
//     state_memory_pool_.Free(static_cast<DrivingState *>(state));
// }

// int Driving::numActiveObs() const {
//     return obs_memory_pool_.num_allocated();
// }

// int Driving::numActiveStates() const {
//     return state_memory_pool_.num_allocated();
// }

// }  // namespace driving
// }  // namespace ipft
