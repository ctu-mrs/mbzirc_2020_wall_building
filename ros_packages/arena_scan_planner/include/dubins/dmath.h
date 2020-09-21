/*
 * dmath.h
 *
 *  Created on: 20. 7. 2014
 *      Author: Petr Vana
 */

#pragma once

#include <stdlib.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>

namespace opendubins {

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884 /* pi use l for long double */
#endif

    const double TOLERANCE = 1e-5;

    inline double myRandom() {
        //return rand() / (myFloat)RAND_MAX;
        return random() / (double) RAND_MAX;
    }

    inline int myIntRandom(const int size) {
        //return rand() % size;
        return random() % size;
    }

    template<typename T>
    constexpr int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    inline double angleToLeft(const double &ang1, const double &ang2) {
        double ret = ang2 - ang1;

        while (ret > 2 * M_PI) {
            ret -= 2 * M_PI;
        }
        while (ret < 0) {
            ret += 2 * M_PI;
        }
        return ret;
    }

    inline double angleToRight(const double &ang1, const double &ang2) {
        double ret = ang2 - ang1;

        while (ret < -2 * M_PI) {
            ret += 2 * M_PI;
        }
        while (ret > 0) {
            ret -= 2 * M_PI;
        }

        return ret;
    }

    inline double angleToSide(const double &ang1, const double &ang2, const bool& isLeft){
        return isLeft ? angleToLeft(ang1, ang2) : angleToRight(ang1, ang2);
    }

    inline double checkToleranceLeft(double turn) {
        return (turn > 2 * M_PI - TOLERANCE && turn < 2 * M_PI + TOLERANCE) ? 0 : turn;
    }

    inline double checkToleranceRight(double turn) {
        return (turn < -2 * M_PI + TOLERANCE && turn > -2 * M_PI - TOLERANCE) ? 0 : turn;
    }

    inline void randomOrder(std::vector<int> &order, int size) {
        order.clear();
        for (int i = 0; i < size; i++)
            order.push_back(i);
        std::random_shuffle(order.begin(), order.end(), myIntRandom);
    }

    constexpr int modulo(int a, int b) {
        return (a + b) % b;
    }

    inline double get_cpu_time() {
        return (double) clock() / CLOCKS_PER_SEC;
    }

}
