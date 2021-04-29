#include <glog/logging.h>

#include <cmath>
#include <solver_ipft/interface/point.hpp>

namespace solver_ipft {

bool Point::equals(const Point &p) const {
  throw std::runtime_error("Point::equals not supported!");
  return false;
}

int Point::dimensions() const {
  throw std::runtime_error("Point::dimensions not supported!");
  return -1;
}

double Point::get(int dim) const {
  throw std::runtime_error("Point::get not supported!");
  return std::nan("");
}

void Point::set(const double &p, int dim) {
  throw std::runtime_error("Point::set not supported!");
}

void Point::reset() { throw std::runtime_error("Point::reset not supported!"); }

std::string Point::text() const {
  throw std::runtime_error("Point::text not supported!");
  return std::string("Not implemented");
}

std::vector<double> Point::mean(const std::vector<Point *> &points) {
  CHECK(!points.empty()) << "Point set empty";

  int dimensions = points[0]->dimensions();
  std::vector<double> meanVec(dimensions);

  for (int dim = 0; dim < dimensions; dim++) {
    double dimValue = 0.0;
    for (auto &p : points) {
      dimValue += p->get(dim);
    }
    dimValue = dimValue / points.size();
    meanVec[dim] = dimValue;
  }
  return meanVec;
}

std::vector<double> Point::variance(const std::vector<Point *> &points) {
  std::vector<double> meanVec = Point::mean(points);
  CHECK(points.size() > 1)
      << "Need more than one particle to compute a variance";
  int dimensions = points[0]->dimensions();
  std::vector<double> varianceVec(dimensions);

  for (int dim = 0; dim < dimensions; dim++) {
    double dimVarianceValue = 0.0;
    for (auto &p : points) {
      double x = p->get(dim) - meanVec[dim];
      dimVarianceValue += x * x;
    }
    // this is how var() is implemented in julia -> in literature: also
    // (points.size()-1) (correction factor)
    dimVarianceValue = dimVarianceValue / (points.size());
    varianceVec[dim] = dimVarianceValue;
  }
  return varianceVec;
}

double Point::epsilon() const { return std::nan("1"); }

} // namespace solver_ipft
