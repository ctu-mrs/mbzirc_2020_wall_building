/*
 * dubins.cpp
 *
 *  Created on: 20. 7. 2014
 *      Author: Petr Vana
 */

#include "dubins/dubins.h"

#include <iomanip>

using namespace std;

namespace opendubins {

Dubins::Dubins() :
		len1(NAN), len2(NAN), len3(NAN), radius1(NAN), radius2(NAN), isCCC(false), length(numeric_limits<double>::max()), type(DType::Unknown) {
}

Dubins::Dubins(const Line &line) :
		Dubins(line.getStart(), false, 0, line.getLength(), 0, 1) {
	type = DType::Unknown;
}

Dubins::Dubins(State pos, bool isCCC, double le1, double le2, double le3, double rad) {
	start = pos;
	this->radius1 = rad;
	this->radius2 = rad;
	this->isCCC = isCCC;
	len1 = le1;
	len2 = le2;
	len3 = le3;
	calculateLength();
	end = getSecondArc().getEnd();
}

Dubins::~Dubins() {
}

bool Dubins::check() {
	return end.point.distance(getSecondArc().getEnd().point) < ((5 + 2 * length) * TOLERANCE);
}

double Dubins::getTimeLength(double max_velocity, double max_acceleration) const {
	//a = v^2/r
	const double v1circ = sqrt(max_acceleration * radius1);
	const double v2circ = sqrt(max_acceleration * radius2);
	//cout << "getTimeLength init_velocity " << v1circ << " final_velocity " << v2circ << std::endl;
	const double dist_middle = getLen2Dist();
	//cout << "getTimeLength dist_middle " << dist_middle << std::endl;
	double time_middle = 0;

	//if (!isCCC) {
	const double time_from_init_to_final_vel = fabs(v2circ - v1circ) / max_acceleration;
	const double dist_from_init_to_final_vel = 0.5 * (v2circ + v1circ) * time_from_init_to_final_vel;

	const double time_from_init_to_max_vel = (max_velocity - v1circ) / max_acceleration;
	const double dist_from_init_to_max_vel = 0.5 * (max_velocity + v1circ) * time_from_init_to_max_vel;

	const double time_from_max_to_final_vel = (max_velocity - v2circ) / max_acceleration;
	const double dist_from_max_vel_to_final = 0.5 * (max_velocity + v2circ) * time_from_max_to_final_vel;

	//cout << "getTimeLength time_from_init_to_max_vel " << time_from_init_to_max_vel << std::endl;
	//cout << "getTimeLength time_from_max_to_final_vel " << time_from_max_to_final_vel << std::endl;

	//cout << "getTimeLength dist_from_init_to_max_vel " << dist_from_init_to_max_vel << std::endl;
	//cout << "getTimeLength dist_from_max_vel_to_final " << dist_from_max_vel_to_final << std::endl;

	if (dist_middle < dist_from_init_to_final_vel) {
		//no time to change speed between init and final velocities
		//cout << "notime to acc/decc between init and final velocities " << v1circ << " " << v2circ << std::endl;
		time_middle = DBL_MAX;
	} else {

		if (dist_middle >= dist_from_init_to_max_vel + dist_from_max_vel_to_final) {
			//time to acc to max speed
			const double time_max_vel = (dist_middle - dist_from_init_to_max_vel - dist_from_max_vel_to_final) / max_velocity;
			time_middle = time_from_init_to_max_vel + time_max_vel + time_from_max_to_final_vel;
		} else {
			//no time to acc to max speed
			const double time_init_accel_final_decc = fabs(v1circ - v2circ) / max_acceleration;

			//cout << "getTimeLength time_init_accel_final_decc " << time_init_accel_final_decc << std::endl;
			const double dist_init_accel = time_init_accel_final_decc * (v1circ + v2circ) * 0.5;
			//cout << "getTimeLength dist_init_accel " << dist_init_accel << std::endl;
			const double larger_velocity = max(v1circ, v2circ);
			//cout << "getTimeLength larger_velocity " << larger_velocity << std::endl;
			const double dist_acc_decc = dist_middle - dist_init_accel;
			const double time_to_possible_max_vel = (-larger_velocity + sqrt(larger_velocity * larger_velocity + max_acceleration * dist_acc_decc))
					/ max_acceleration;
			//cout << "getTimeLength time_to_possible_max_vel " << time_to_possible_max_vel << std::endl;
			time_middle = 2 * time_to_possible_max_vel + time_init_accel_final_decc;
		}
	}
	/*
	 } else {
	 if (radius1 != radius2) {
	 cout << "isCCC is not feasible if raddi are different " << radius1 << " " << radius2 << std::endl;
	 cout << "isCCC len 2 is " << getLen2Dist() << std::endl;
	 time_middle = DBL_MAX;
	 //exit(1);
	 } else {
	 time_middle = getLen2Dist() / v1circ;
	 }
	 }
	 */

	//cout << "getTimeLength part times " << (getLen1Dist() / v1circ) << " " << time_middle << " " << (getLen3Dist() / v2circ) << std::endl;
	double time = getLen1Dist() / v1circ + time_middle + getLen3Dist() / v2circ;
	return time;
}

State Dubins::getState(double len) const {
	Arc path1 = getFirstArc();
	double l1 = path1.getLength();
	if (len < l1) {
		return path1.getState(len);
	}

	double l2;
	if (isCCC) {
		Arc ca = getCenterArc();
		l2 = ca.getLength();
		if (len < l1 + l2) {
			return ca.getState(len - l1);
		}
	} else {
		Line cl = getCenter();
		l2 = cl.getLength();
		if (len < l1 + l2) {
			return cl.getState(len - l1);
		}
	}

	Arc path3 = getSecondArc();
	return path3.getState(len - l1 - l2);
}

StateAtDistance Dubins::getClosestStateAndDistance(const Point &p) const {
	StateAtDistance closest;

	closest.state = start;
	closest.distance = 0;

	auto a1c = getFirstArc().getClosestStateAndDistance(p);
	if (a1c.state.point.distance(p) < closest.state.point.distance(p)) {
		closest = a1c;
	}

	auto a2c = getSecondArc().getClosestStateAndDistance(p);
	if (a2c.state.point.distance(p) < closest.state.point.distance(p)) {
		closest = a2c;
	}

	StateAtDistance a3c;
	if (isCCC) {
		Arc ca = getCenterArc();
		a3c = ca.getClosestStateAndDistance(p);
	} else {
		Line cl = getCenter();
		a3c = cl.getClosestStateAndDistance(p);
	}

	if (a3c.state.point.distance(p) < closest.state.point.distance(p)) {
		closest = a3c;
	}

	return closest;
}

Arc Dubins::getFirstArc() const {
	auto st = start;
	return Arc(st, len1, radius1);
}

Line Dubins::getCenter() const {
	return Line(getFirstArc().getEnd(), len2);
}

Arc Dubins::getCenterArc() const {
	State p = getFirstArc().getEnd();
	return Arc(p, len2, radius1);
}

Arc Dubins::getSecondArc() const {
	State st;
	if (isCCC) {
		st = getCenterArc().getEnd();
	} else {
		Point ps = getCenter().p2;
		st = State(ps, start.ang + len1);
	}
	return Arc(st, len3, radius2);
}

StateAtDistance Dubins::intersectLine(const Line &line) const {

	StateAtDistance p = getFirstArc().intersectionPoint(line);
	if (!p.state.invalid()) {
		return p;
	}

	if (isCCC) {
		p = getCenterArc().intersectionPoint(line);
		if (!p.state.invalid()) {
			return p;
		}
	} else {
		p = getCenter().intersectionPoint(line);
		if (!p.state.invalid()) {
			return p;
		}
	}

	p = getSecondArc().intersectionPoint(line);
	if (!p.state.invalid()) {
		return p;
	}

	return StateAtDistance(State(), 0);
}

bool Dubins::intersectLineBool(const Line &line) const {
	if (!getFirstArc().intersectionPoint(line).state.invalid()) {
		return true;
	}

	if (isCCC) {
		if (!getCenterArc().intersectionPoint(line).state.invalid()) {
			return true;
		}
	} else {
		if (!getCenter().intersectionPoint(line).state.invalid()) {
			return true;
		}
	}

	if (!getSecondArc().intersectionPoint(line).state.invalid()) {
		return true;
	}

	return false;
}

ostream &operator<<(ostream &os, const Dubins &d) {
	os << setprecision(5);
	os << "Dubins maneuver " << d.start << " --> " << d.end << endl << (d.isCCC ? "CCC" : "CSC") << " type " << "\tlen = " << d.getLength() << "\t("
			<< "n1 = " << d.getLen1() << ", n2 = " << d.getLen2() << ", n3 = " << d.getLen3() << ")";

	return os;
}
}
