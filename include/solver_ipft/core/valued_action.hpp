#pragma once

#include <solver_ipft/interface/spaces.hpp>

namespace solver_ipft {

class Value;

/* -------------------------------------------------------------------------- */
/*                             ValuedAction class                             */
/* -------------------------------------------------------------------------- */

struct ValuedAction {
    Action action_;
    std::unique_ptr<Value> value_;
    int count_;
    // index no action / invalid action constructor
    ValuedAction();
    // no value/count constructor
    explicit ValuedAction(Action _action);
    // regular constructor
    ValuedAction(Action _action, std::unique_ptr<Value>&& _value, int _count);
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

} // namespace solver_ipft