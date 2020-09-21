/*
 * MathCommon.cpp
 *
 *  Created on: 16. 11. 2014
 *      Author: Robert Pěnička
 */

#include <dubins/MathCommon.h>

bool randSeeded = false;

double randDoubleMinMax(double min, double max) {
	if (!randSeeded) {
		srand(time(NULL));
		randSeeded = true;
	}
	double random = ((double) rand() / (double) RAND_MAX);
	random = min + random * (max - min);
	return random;
}

int randIntMinMax(int min, int max) {
	if (!randSeeded) {
		srand(time(NULL));
		randSeeded = true;
	}
	int random = min + (int) (((double) rand() / ((double) RAND_MAX + 1)) * (max - min + 1));
	return random;
}

double normalizeAngle(double angle, double min, double max) {
	double normalized = angle;
	while (normalized < min) {
		normalized += M_2PI;
	}
	while (normalized > max) {
		normalized -= M_2PI;
	}
	return normalized;
}

int sgn(int val) {
	if (val > 0) {
		return 1;
	} else if (val < 0) {
		return -1;
	} else {
		return 0;
	}
}

double sgn(double val) {
	if (val > 0) {
		return 1;
	} else if (val < 0) {
		return -1;
	} else {
		return 0;
	}
}

float sgn(float val) {
	if (val > 0) {
		return 1;
	} else if (val < 0) {
		return -1;
	} else {
		return 0;
	}
}

std::vector<double> range(double min, double max, double step) {
	std::vector<double> range_values;
	if (min <= max) {
		for (double val = 0; val < max; val = val + step) {
			range_values.push_back(val);
		}
		range_values.push_back(max);
	}
	return range_values;
}
