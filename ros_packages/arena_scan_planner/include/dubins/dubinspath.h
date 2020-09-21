/*
 * dubinspath.h
 *
 *  Created on: 20. 6. 2016
 *      Author: Petr Vana
 */

#pragma once

#include "path.h"
#include "dubins.h"
#include "stateatdistance.h"

namespace opendubins {

    class DubinsPath : public Path {
    public:

        std::vector<Dubins> path;

        State getState(double len) const;
        double getLength() const;
        bool intersect(Line line) const;

        // todo
        StateAtDistance getClosestStateAndDistance(const Point &p) const;
        StateAtDistance intersectionPoint(Line line) const;

        std::vector<Point> interpolate(double step) const;

        inline void add(const Dubins & d) {
            path.push_back(d);
        }

    };

    inline std::ostream &operator<<(std::ostream &os, const DubinsPath &d) {
        // TODO
        //os << "Line from " << d.p1 << " to " << d.p2;
        return os;
    }

}


