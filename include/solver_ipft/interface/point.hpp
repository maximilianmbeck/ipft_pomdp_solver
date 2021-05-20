#pragma once

#include <iostream>
#include <string>

#include <solver_ipft/util/memorypool.hpp>
namespace solver_ipft {
/**
 * @brief An interface defining a Point in a metric space needs an equals method
 */
class Point : public MemoryObject {
public:
    Point() : MemoryObject() {
    }
    virtual ~Point() = default;

    Point(const Point&) = default;
    Point(Point&&) = default;
    Point& operator=(const Point&) = default;
    Point& operator=(Point&&) = default;

    /**
     * @brief Check if two points are equal
     */
    virtual bool equals(const Point& p) const;

    /**
     * @brief Return the number of dimensions
     */
    virtual int dimensions() const;

    /**
     * @brief Get the element at dimension
     */
    virtual double get(int dim) const;

    /**
     * @brief Set the element at dimension
     */
    virtual void set(const double& p, int dim);

    /**
     * @brief Clear the point object
     */
    virtual void reset();

    /**
     * @brief Returns contents of the point
     */
    virtual std::string text() const;

    /**
     * @brief Stream contents of the point
     */
    friend std::ostream& operator<<(std::ostream& os, const Point& p) {
        os << p.text();
        return os;
    }

    /**
     * @brief Calculate the mean value of the points (over all dimensions)
     */
    static std::vector<double> mean(const std::vector<Point*>& points);

    /**
     * @brief Calculate the variance of the points (over all dimensions)
     */
    static std::vector<double> variance(const std::vector<Point*>& points);

private:
    /**
     * @brief Tolerance for comparing two points
     */
    virtual double epsilon() const;
};

} // namespace solver_ipft