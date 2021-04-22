#pragma once

#include "solver_ipft/interface/spaces.hpp"

namespace solver_ipft {

class Value;

/* -------------------------------------------------------------------------- */
/*                             ValuedAction class                             */
/* -------------------------------------------------------------------------- */

struct ValuedAction {
    Action action_;
    Value* value_;
    int count_;
    // TODO: store model as well and print action value instead of action index
    // no action / invalid action constructor
    ValuedAction();
    // no value/count constructor
    ValuedAction(Action _action);
    // regular constructor
    ValuedAction(Action _action, Value* _value, int _count);
    // copy constructor
    ValuedAction(const ValuedAction& other);
    // assignment operator
    ValuedAction& operator=(const ValuedAction& rhs);
    // move copy constructor
    ValuedAction(ValuedAction&& other) noexcept;
    // move assigment operator
    ValuedAction& operator=(ValuedAction&& rhs) noexcept;
    // destructor
    virtual ~ValuedAction();

    std::string text() const;

    friend std::ostream& operator<<(std::ostream& os, const ValuedAction& va);
};

}  // namespace solver_ipft