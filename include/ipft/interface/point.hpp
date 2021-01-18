#pragma once

#include <iostream>
#include <string>

#include "ipft/util/memorypool.hpp"
namespace solver_ipft {
/**
 * @brief An interface defining a Point in a metric space
 * needs an equals method, and a metric defined on the space
 *
 */
class Point : public MemoryObject {
   public:
    Point(){};
    virtual ~Point(){};

    virtual bool equals(const Point& p) const;

    virtual int dimensions() const;

    virtual double get(int dim) const;

    virtual void set(const double& p, int dim);

    virtual void reset();

    // don't do this Points should be only allocated through the POMDP model
    // virtual Point* clone() const = 0;

    virtual std::string text() const;

    friend std::ostream& operator<<(std::ostream& os, const Point& p) {
        os << p.text();
        return os;
    }

    static std::vector<double> mean(const std::vector<Point*>& points);

    static std::vector<double> variance(const std::vector<Point*>& points);

   private:
    virtual double epsilon() const;
};

}  // namespace solver_ipft