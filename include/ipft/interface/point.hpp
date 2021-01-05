#pragma once

#include <iostream>
#include <string>

#include "ipft/util/memorypool.hpp"
namespace ipft {
/**
 * @brief An interface defining a Point in a metric space
 * needs an equals method, and a metric defined on the space
 *
 */
class Point : public MemoryObject {
   public:
    Point(){};
    virtual ~Point(){};

    virtual bool equals(const Point& p) const = 0;

    // virtual double distanceTo(const Point& p) const = 0;

    virtual int dimensions() const = 0;

    virtual double get(int dim) const = 0;

    virtual void set(const double& p, int dim) = 0;

    virtual void reset() = 0;

    // virtual bool operator<(const Point& rhs) const = 0;

    // virtual bool operator==(const Point& rhs) const = 0;

    // don't do this Points should be only allocated through the POMDP model
    // virtual Point* clone() const = 0;

    virtual std::string text() const = 0;

    friend std::ostream& operator<<(std::ostream& os, const Point& p) {
        os << p.text();
        return os;
    }

    static std::vector<double> mean(const std::vector<Point*>& points) {
        CHECK(points.size() > 0) << "Point set empty";

        int dimensions = points[0]->dimensions();
        std::vector<double> meanVec(dimensions);

        for (int dim = 0; dim < dimensions; dim++) {
            double dimValue = 0.0;
            for (int i = 0; i < points.size(); i++) {
                dimValue += points[i]->get(dim);
            }
            dimValue = dimValue / points.size();
            meanVec[dim] = dimValue;
        }
        return meanVec;
    }

    static std::vector<double> variance(const std::vector<Point*>& points) {
        std::vector<double> meanVec = Point::mean(points);
        CHECK(points.size() > 1) << "Need more than one particle to compute a variance";
        int dimensions = points[0]->dimensions();
        std::vector<double> varianceVec(dimensions);

        for (int dim = 0; dim < dimensions; dim++) {
            double dimVarianceValue = 0.0;
            for (int i = 0; i < points.size(); i++) {
                double x = points[i]->get(dim) - meanVec[dim];
                dimVarianceValue += x * x;
            }
            // this is how var() is implemented in julia -> in literature: also (points.size()-1) (correction factor)
            dimVarianceValue = dimVarianceValue / (points.size());
            varianceVec[dim] = dimVarianceValue;
        }
        return varianceVec;
    }

   private:
    virtual double epsilon() const = 0;
};

}  // namespace ipft