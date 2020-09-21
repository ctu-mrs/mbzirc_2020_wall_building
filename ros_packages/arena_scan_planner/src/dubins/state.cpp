/*
 * state.cpp
 *
 *  Created on: 18. 7. 2014
 *      Author: Petr Vana
 */

#include "dubins/state.h"

using namespace std;

namespace opendubins {

    State::State() {
        this->point = Point(0, 0);
        this->ang = NAN;
    }

    State::State(Point point, double angle) {
        this->point = point;
        this->ang = angle;
    }

    State::State(double x, double y, double angle) {
        this->point = Point(x, y);
        this->ang = angle;
    }

    void State::random(double interval) {
        this->point = Point(myRandom() * interval, myRandom() * interval);
        this->ang = myRandom() * 2 * M_PI;
    }

    Vector State::getNormalizedDirection() const {
        return Vector(cos(ang), sin(ang));
    }

    ostream &operator<<(ostream &os, const State &pos) {
        Point p = pos.getPoint();
        os << "(" << p.x << ", " << p.y << ", " << pos.getAngle() << ")";
        return os;
    }

}

