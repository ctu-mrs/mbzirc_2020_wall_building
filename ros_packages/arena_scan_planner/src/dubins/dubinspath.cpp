/*
 * dubinspath.cpp
 *
 *  Created on: 20. 6. 2016
 *      Author: Petr Vana
 */

#include "dubins/dubinspath.h"

namespace opendubins {

    State DubinsPath::getState(double len) const {
        double sum = 0;
        for (auto & d: path) {
            double diff = len - sum;
            if(diff < d.length){
                return d.getState(diff);
            }
            sum += d.length;
        }
        return path.back().getEnd();
    }

    double DubinsPath::getLength() const{
        double sum = 0;
        for (auto & d: path) {
            sum += d.length;
        }
        return sum;
    }

    bool DubinsPath::intersect(Line line) const{
        for (auto & d: path) {
            if(d.intersectLineBool(line)){
                return true;
            }
        }
        return false;
    }

    // todo
    StateAtDistance DubinsPath::getClosestStateAndDistance(const Point &p) const{}
    StateAtDistance DubinsPath::intersectionPoint(Line line) const{}

    std::vector<Point> DubinsPath::interpolate(double step) const {
        std::vector<Point> pnts;

        double speed = 0;

        for(double distance = 0; distance < getLength(); distance += speed){
            pnts.push_back(getState(distance).point);
            speed += step * 0.1;
            if(speed > step){
                speed = step;
            }
        }

        return pnts;
    }
}
