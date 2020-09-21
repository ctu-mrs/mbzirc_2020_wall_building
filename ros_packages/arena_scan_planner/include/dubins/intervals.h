/*
 * angleinterval.h
 *
 *  Created on: Dec 4, 2015
 *      Author: Petr Vana
 */

#pragma once

#include <algorithm>

namespace opendubins {

    struct Interval {

        // right angle from the interval
        // startAngle \in <0,2*PI)
        double rightDir;

        // diff between left and right angle <0,2*PI>
        // diff = 0    >> single direction
        // diff = 2*PI >> every possible dirrection
        double diff;

        Interval() : diff(NAN), rightDir(NAN) { };

        Interval(double rightDir, double diff) : diff(diff), rightDir(rightDir) { };

        inline bool operator<(const Interval &o) const {
            return rightDir < o.rightDir;
        }

        static std::vector<Interval> mergeIntervals(std::vector<Interval> input){
            decltype(input) merged;
            decltype(input) twice;
            decltype(input) filtered;

            if(input.size() > 0) {
                // sort intervals
                sort(input.begin(), input.end());

                // make second copy of intervals from [0, 2PI] to [2PI, 4PI]
                twice = input;
                for (auto i : input) {
                    i.rightDir += 2 * M_PI;
                    twice.push_back(i);
                }

                // merge intervals in interval [0, 4PI]
                // last interval can end behind 4PI (but no problem here)
                double from = twice[0].rightDir;
                double to = from + twice[0].diff;
                for (int i = 1; i < twice.size(); ++i) {
                    const auto &it = twice[i];
                    if (it.rightDir <= to) {
                        to = std::max(to, (it.rightDir + it.diff));
                    } else {
                        // cannot use constructor directly - a normalization is included
                        Interval ai;
                        ai.rightDir = from;
                        ai.diff = to - from;
                        merged.push_back(ai);
                        from = it.rightDir;
                        to = it.rightDir + it.diff;
                    }
                }
                // cannot use constructor directly - a normalization is included
                Interval ai;
                ai.rightDir = from;
                ai.diff = to - from;
                merged.push_back(ai);

                // filter interval and reduce them to interval [2PI, 4PI]
                for(auto i : merged){
                    const double M = 2*M_PI;
                    auto a = i.rightDir - M;
                    auto b = a + i.diff;
                    if(a < M && b > 0){
                        a = std::max(a, 0.0);
                        b = std::min(b, M);
                        filtered.push_back(Interval(a, b-a));
                    }
                }
            }

            return filtered;
        }
    };

}
