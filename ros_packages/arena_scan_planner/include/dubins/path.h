/*
 * path.h
 *
 *  Created on: Aug 1, 2015
 *      Author: Petr Vana
 */

#pragma once

#include "state.h"
#include "stateatdistance.h"

namespace opendubins {

    class Path {
    public:
        Path();

        virtual ~Path();

    public:
        virtual double getLength() const = 0;

        virtual State getState(double len) const = 0;

        virtual State getStart() const;

        virtual State getEnd() const;

        /**
         * Get the closest state to point p on the path according to the Euclidean metric.
         * It invokes function getClosestIntersection(const Point &p) and return only state.
         */
        virtual State getClosestState(const Point &p) const;

        /**
         * Find the closest state to point p on the path according to the euclidean metric.
         * It return Intersection (state and distance).
         */
        virtual StateAtDistance getClosestStateAndDistance(const Point &p) const = 0;
    };

}
