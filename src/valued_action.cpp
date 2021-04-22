#include "solver_ipft/core/valued_action.hpp"

#include <iomanip>
#include <sstream>

#include "solver_ipft/core/solver.hpp"

namespace solver_ipft {
/* -------------------------------------------------------------------------- */
/*                             ValuedAction class                             */
/* -------------------------------------------------------------------------- */

// no action / invalid action constructor
ValuedAction::ValuedAction() : action_(-1), value_(nullptr), count_(-1) {
}
// no value / count constructor
ValuedAction::ValuedAction(Action _action) : action_(_action), value_(nullptr), count_(-1) {
}
// regular constructor
ValuedAction::ValuedAction(Action _action, Value* _value, int _count)
    : action_(_action), value_(_value), count_(_count) {
}
// copy constructor
ValuedAction::ValuedAction(const ValuedAction& other) : action_(other.action_), count_(other.count_) {
    if (other.value_ != nullptr) {
        value_ = other.value_->clone();
    } else {
        value_ = nullptr;
    }
}
// assignment operator
ValuedAction& ValuedAction::operator=(const ValuedAction& rhs) {
    if (this != &rhs) {
        this->action_ = rhs.action_;
        this->count_ = rhs.count_;
        if (value_ != nullptr) {
            delete value_;
        }
        if (rhs.value_ != nullptr) {
            this->value_ = rhs.value_->clone();
        }
    }
    return *this;
}
// move copy constructor
ValuedAction::ValuedAction(ValuedAction&& other) noexcept {
    this->action_ = other.action_;
    this->count_ = other.count_;
    this->value_ = other.value_;
    other.value_ = nullptr;
}
// move assigment operator
ValuedAction& ValuedAction::operator=(ValuedAction&& rhs) noexcept {
    if (this != &rhs) {
        std::swap(this->value_, rhs.value_);
        this->action_ = rhs.action_;
        this->count_ = rhs.count_;
    }
    return *this;
}
// destructor
ValuedAction::~ValuedAction() {
    if (value_ != nullptr) {
        delete value_;
    }
}

/* -------------------------- ValuedAction printers ------------------------- */

std::string ValuedAction::text() const {
    using namespace std;
    stringstream ss;
    if (this->value_ != nullptr) {
        ss << "[ValuedAction: Act(" << this->action_ << ") " << *(this->value_) << " Count(" << this->count_ << ")]";
    } else {
        ss << "[ValuedAction: Act(" << this->action_ << ") [Value: no value] Count(" << this->count_ << ")]";
    }
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const ValuedAction& va) {
    os << std::setfill(' ') << std::left << std::setw(70) << va.text();
    return os;
}
}  // namespace solver_ipft
