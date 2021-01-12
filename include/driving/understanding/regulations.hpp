#pragma once

#include <vector>

namespace driving {

namespace understanding {

class TrafficRules {
    struct Stopline {
        double distanceToStopline;  // in Frenet-CS
        bool hasToStop;
    };

    struct Speedlimit {
        double speedlimit;     // in m/s
        double inEffectUntil;  // in Frenet-CS
    };

   protected:
    std::vector<Stopline> stoplines;
    std::vector<Speedlimit> speedlimits;

   public:
    // constructor here
};

}  // namespace understanding

}  // namespace driving
