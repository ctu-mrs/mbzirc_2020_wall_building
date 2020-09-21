/*
 * dtype.h - Types of Dubins maneuver
 *
 *  Created on: Mar 6, 2016
 *      Author: Petr Vana
 */

#pragma once

#include <ostream>

namespace opendubins {

enum struct DType {
	Unknown,

	RSR, LSL, RSL, LSR, RLR, LRL,

	// Dubins Interval Problem ----------------------------------------
	// CSC and CCC maneuvers used from Dubins maneuver
	DIP_S,
	DIP_Rp,
	DIP_Lp,
	DIP_RS,
	DIP_LS,
	DIP_SR,
	DIP_SL,
	DIP_RLp,
	DIP_LRp,
	DIP_RpL,
	DIP_LpR,

	// special cases - this should never happen
	DIP_LpRp,
	DIP_RpLp,

	// Generalized Dubins Interval Problem
	GDIP_NO,
	GDIP_S,
	GDIP_R,
	GDIP_RS,
	GDIP_L,
	GDIP_LS,
	GDIP_SR,
	GDIP_SL,
	GDIP_RSR,
	GDIP_LSL,
	GDIP_RSL,
	GDIP_LSR,
	GDIP_LRL,
	GDIP_RLR
};

inline std::ostream& operator<<(std::ostream& s, DType type) {
	switch (type) {
	case DType::Unknown:
		s << "Unknown";
		break;

	case DType::RSR:
		s << "RSR";
		break;
	case DType::LSL:
		s << "LSL";
		break;
	case DType::RSL:
		s << "RSL";
		break;
	case DType::LSR:
		s << "LSR";
		break;
	case DType::RLR:
		s << "RLR";
		break;
	case DType::LRL:
		s << "LRL";
		break;

	default:
		s << "Invalid";
		break;
	}
	return s;
}
}
