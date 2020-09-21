/*
 * dubins.h
 *
 *  Created on: 20. 7. 2014
 *      Author: Petr Vana
 */

#pragma once

#include "dtype.h"
#include "line.h"
#include "arc.h"
#include "angleinterval.h"
#include "circle.h"
#include <cfloat>

#define USE_XpXp 0

namespace opendubins {

class Dubins: public Path {

private:
	// in radians (positive - left, negative - right) or in meters (straight)
	double len1, len2, len3;

	DType type;

public:

	State start;
	State end;

	double radius1;
	double radius2;

	bool isCCC;

	double length;

	// default constructor
	Dubins();

	// create Dubins from a straight line (radius = 1)
	Dubins(const Line& line);

	// classical Dubins maneuver
	Dubins(const State& newStart, const State& newEnd, const double& newRadius);

	// Dubins maneuver with diff radii
	Dubins(const State& from, const State& to, const double& radius1, const double& radius2);

	// Dubins Interval Problem (DIP)
	Dubins(AngleInterval from, AngleInterval to, double radius);

	// Generalited Dubins Interval Problem (GDIP)
	// diff1 - neighborhood around the start state
	// diff2 - neighborhood around the end state
	Dubins(AngleInterval from, AngleInterval to, double radius, double diff1, double diff2);

	// Generalited Dubins Interval Problem (GDIP)
	// diff - neighborhood around the end state
	Dubins(AngleInterval from, AngleInterval to, double radius, double diff2);

	Dubins(State pos, bool isCCC, double le1, double le2, double le3, double radius);

	virtual ~Dubins();

	bool check();

	virtual State getState(double len) const;

	virtual StateAtDistance getClosestStateAndDistance(const Point &p) const;

	Arc getFirstArc() const;

	Line getCenter() const;

	Arc getCenterArc() const;

	Arc getSecondArc() const;

	StateAtDistance intersectLine(const Line &line) const;

	bool intersectLineBool(const Line &line) const;

	inline double getLength() const {
		return length;
	}

	double getTimeLength(double max_velocity, double max_acceleration) const;

	inline bool getIsCCC() const {
		return isCCC;
	}

	inline double getLen1() const {
		return len1;
	}

	inline double getLen1Dist() const {
		return fabs(len1) * radius1;
	}

	inline double getLen2() const {
		return len2;
	}

	inline double getLen2Dist() const {
		if (isCCC) {
			return fabs(len2) * radius1;
		} else {
			return len2;
		}
	}

	inline double getLen3() const {
		return len3;
	}

	inline double getLen3Dist() const {
		return fabs(len3) * radius2;
	}

	inline double getRadius1() const {
		return radius1;
	}

	inline double getRadius2() const {
		return radius2;
	}

	inline void calculateLength() {
		if (isCCC) {
			length = radius1 * (fabs(len1) + fabs(len2)) + radius2 * fabs(len3);
		} else {
			length = len2 + radius1 * fabs(len1) + radius2 * fabs(len3);
		}
	}

	inline double getTotalTurning() {
		return fabs(len1) + fabs(len3);
	}

	inline DType getType() {
		return type;
	}

private:

	void init(State from, State to);

	void initIntervalProblem(AngleInterval from, AngleInterval to);

	void initGDIP(AngleInterval from, AngleInterval to, double diff);
	void initGDIP(AngleInterval from, AngleInterval to, double diff1, double diff2);

	void GDIP_S(const AngleInterval& from, const AngleInterval& to, const double &diff2);
	void GDIP_S(const AngleInterval& from, const Point& to, const double &diff2);
	void GDIP_S_onEdge(const AngleInterval& from, const Point& to, const double& diff2, double dir);

	void GDIP_CS(const State& from, const AngleInterval& to, const double &diff2, bool isLeft);
	void GDIP_CS_onEdge(const State& from, const AngleInterval& to, const double& diff2, double dir,
			const bool& isLeft);

	void GDIP_SC(const AngleInterval& from, const AngleInterval& to, const double& diff2);

	void GDIP_CSC_same_side(const State &from, const State &to, const double &diff2, const bool &isLeft);
	void GDIP_CSC_diff_side(const State &from, const State &to, const double &diff2, const bool &isLeft);

	static void Dubins_StateToCircle(const State& from, const Circle& goal, const double& radius, const bool& isLeft,
			double& turn, double& straight, double& length);

};

std::ostream &operator<<(std::ostream &os, const Dubins &d);

}
