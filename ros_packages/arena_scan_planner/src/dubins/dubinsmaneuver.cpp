/*
 * dubinsmaneuver.cpp - Shortest path connectint two states
 *
 *  Created on: Mar 8, 2016
 *      Author: Petr Vana
 */

#include "dubins/dubins.h"

namespace opendubins {

Dubins::Dubins(const State& newStart, const State& newEnd, const double& newRadius) {
	radius1 = newRadius;
	radius2 = newRadius;
	length = std::numeric_limits<double>::max();
	type = DType::Unknown;
	//std::cout << "Dubins constructor" << std::endl;
	init(newStart, newEnd);
}

Dubins::Dubins(const State& newStart, const State& newEnd, const double& newRadius1, const double& newRadius2) {
	radius1 = newRadius1;
	radius2 = newRadius2;
	length = std::numeric_limits<double>::max();
	type = DType::Unknown;
	//std::cout << "Dubins constructor" << std::endl;
	init(newStart, newEnd);
}

void Dubins::init(State from, State to) {
	bool used = false;

	//std::cout << "radius1 " << radius1 << std::endl;
	//std::cout << "radius2 " << radius2 << std::endl;
	double sum_radii = radius1 + radius2;
	double diff_radii = radius1 - radius2;

	Vector dir1 = from.getNormalizedDirection(); //vector of init heading
	Vector dir2 = to.getNormalizedDirection(); //vector of end heading

	Vector dir1radius = dir1 * radius1;
	Vector dir2radius = dir2 * radius2;

	Point c1left = from.point + dir1radius.left();
	Point c1right = from.point + dir1radius.right();

	Point c2left = to.point + dir2radius.left();
	Point c2right = to.point + dir2radius.right();

	double n1, n2, n3, nLength, centerDistance;

	//std::cout << std::endl;
	//std::cout << std::endl;
	//std::cout << std::endl;

	// RSR - maneuver
	//std::cout << "RSR " << std::endl;
	Vector diff_centers = c2right - c1right;
	centerDistance = diff_centers.length();

	double angle_tangent = 0;
	if (centerDistance > 0) {
		angle_tangent = asin(diff_radii / diff_centers.length());
	}

	//std::cout << "centerDistance " << centerDistance << std::endl;
	//std::cout << "c1right = [" << c1right.x << " , " << c1right.y << "]" << std::endl;
	//std::cout << "c2right = [" << c2right.x << " , " << c2right.y << "]" << std::endl;

	double to_tangent_angle = diff_centers.left().getAngle() - angle_tangent;

	//std::cout << "to_tangent_angle " << to_tangent_angle << std::endl;

	Vector center_to_start_end = Vector(to_tangent_angle);

	Point start_line = c1right + radius1 * center_to_start_end;
	Point end_line = c2right + radius2 * center_to_start_end;

	//std::cout << "start_line = [" << start_line.x << ", " << start_line.y << "]" << std::endl;
	//std::cout << "end_line = [" << end_line.x << ", " << end_line.y << "]" << std::endl;

	Vector diff = end_line - start_line;
	double ang = diff.getAngle();
	n2 = diff.length();
	if (n2 == 0) {
		ang = atan2(start_line.y - c1right.y, start_line.x - c1right.x) - M_PI / 2.0;
	}

	//std::cout << "ang " << ang << std::endl;

	n1 = angleToRight(from.ang, ang);

	n3 = angleToRight(ang, to.ang);

	n1 = checkToleranceRight(n1);
	n3 = checkToleranceRight(n3);

	nLength = n2 + radius1 * fabs(n1) + radius2 * fabs(n3);

	if (nLength < length) {
		len1 = n1;
		len2 = n2;
		len3 = n3;
		length = nLength;
		used = true;
		isCCC = false;
		type = DType::RSR;

		//std::cout << std::endl;

		//std::cout << "use RSR dubins " << std::endl;
		//std::cout << "len1 " << len1 << std::endl;
		//std::cout << "len2 " << len2 << std::endl;
		//std::cout << "len3 " << len3 << std::endl;
	}

	// LSL - maneuver
	//std::cout << "LSL " << std::endl;
	diff_centers = c2left - c1left;
	centerDistance = diff_centers.length();

	angle_tangent = 0;
	if (centerDistance > 0) {
		angle_tangent = asin(diff_radii / diff_centers.length());
	}
	to_tangent_angle = diff_centers.right().getAngle() + angle_tangent;

	center_to_start_end = Vector(to_tangent_angle);

	start_line = c1left + radius1 * center_to_start_end;
	end_line = c2left + radius2 * center_to_start_end;

	diff = end_line - start_line;
	ang = diff.getAngle();
	n2 = diff.length();
	if (n2 == 0) {
		ang = atan2(start_line.y - c1left.y, start_line.x - c1left.x) + M_PI / 2.0;
	}

	//std::cout << "c1left = [" << c1left.x << ", " << c1left.y << "]" << std::endl;
	//std::cout << "c2left = [" << c2left.x << ", " << c2left.y << "]" << std::endl;

	n1 = angleToLeft(from.ang, ang);

	n3 = angleToLeft(ang, to.ang);

	n1 = checkToleranceLeft(n1);
	n3 = checkToleranceLeft(n3);

	nLength = n2 + radius1 * fabs(n1) + radius2 * fabs(n3);

	if (nLength < length) {

		//std::cout << "c1left = [" << c1left.x << ", " << c1left.y << "]" << std::endl;
		//std::cout << "c2left = [" << c2left.x << ", " << c2left.y << "]" << std::endl;

		len1 = n1;
		len2 = n2;
		len3 = n3;
		length = nLength;
		used = true;
		isCCC = false;
		type = DType::LSL;
	}

	// LSR - maneuver
	//std::cout << "LSR " << std::endl;
	diff = c2right - c1left;
	centerDistance = diff.length();

	if (centerDistance * (1.0) >= sum_radii) {
		double alpha = asin(fmin(1, sum_radii / centerDistance));
		double centerAngle = atan2(c2right.y - c1left.y, c2right.x - c1left.x) + alpha;
		n2 = sqrt(fmax(0, centerDistance * centerDistance - sum_radii * sum_radii));

		// normalize angle
		n1 = angleToLeft(from.ang, centerAngle);
		n3 = angleToRight(centerAngle, to.ang);

		n1 = checkToleranceLeft(n1);
		n3 = checkToleranceRight(n3);

		nLength = n2 + radius1 * fabs(n1) + radius2 * fabs(n3);

		if (nLength < length) {
			len1 = n1;
			len2 = n2;
			len3 = n3;
			length = nLength;
			used = true;
			isCCC = false;
			type = DType::LSR;
		}
	}

// RSL - maneuver

	diff = c2left - c1right;
	centerDistance = diff.length();

	if (centerDistance * (1.0) >= sum_radii) {
		//std::cout << "RSL " << std::endl;
		double alpha = asin(fmin(1, sum_radii / centerDistance));
		double centerAngle = atan2(c2left.y - c1right.y, c2left.x - c1right.x) - alpha;
		n2 = sqrt(fmax(0, centerDistance * centerDistance - sum_radii * sum_radii));

		// normalize angle
		n1 = angleToRight(from.ang, centerAngle);
		n3 = angleToLeft(centerAngle, to.ang);

		n1 = checkToleranceRight(n1);
		n3 = checkToleranceLeft(n3);

		nLength = n2 + radius1 * fabs(n1) + radius2 * fabs(n3);

		if (nLength < length) {
			len1 = n1;
			len2 = n2;
			len3 = n3;
			length = nLength;
			used = true;
			isCCC = false;
			type = DType::RSL;
		}
	}

// CCC maneuver is possible only in case start and end state is close enough
	if ((from.point - to.point).length() <= radius1 * 3 + radius2) {
		//std::cout << "RLR" << std::endl;
		// RLR - maneuver
		diff = c2right - c1right;
		centerDistance = diff.length();
		//std::cout << centerDistance << std::endl;
		if (centerDistance <= radius1 * 3 + radius2) {
			// direction of Vector(S1,S2) to Vector(S1,S3)
			//double alpha = acos(centerDistance / radius / 4);

			//std::cout << "c1right = [" << c1right.x << " , " << c1right.y << "]" << std::endl;
			//std::cout << "c2right = [" << c2right.x << " , " << c2right.y << "]" << std::endl;

			//double alpha = acos(centerDistance / (radius1 * 3 + radius2));
			//std::cout << "centerDistance " << centerDistance << std::endl;
			double alpha = acos(
					(centerDistance * centerDistance + 4 * radius1 * radius1 - sum_radii * sum_radii)
							/ (2 * 2 * radius1 * centerDistance));
			double alpha2 = acos(
					(centerDistance * centerDistance + sum_radii * sum_radii - 4 * radius1 * radius1)
							/ (2 * centerDistance * sum_radii));

			// direction between first and second arc
			double dir12 = diff.getAngle() - M_PI / 2 - alpha;
			// direction between second and third arc
			double dir23 = diff.getAngle() + M_PI / 2 + alpha2;

			n1 = angleToRight(from.ang, dir12);
			n2 = angleToLeft(dir12, dir23);
			n3 = angleToRight(dir23, to.ang);

			nLength = radius1 * (fabs(n1) + fabs(n2)) + radius2 * fabs(n3);
			//std::cout << "nLength " << nLength << std::endl;
			if (nLength < length) {
				isCCC = true;
				len1 = n1;
				len2 = n2;
				len3 = n3;
				length = nLength;
				used = true;
				type = DType::RLR;
			}
		}

		// LRL - maneuver
		diff = c2left - c1left;
		centerDistance = diff.length();

		if (centerDistance <= radius1 * 3 + radius2) {
			//std::cout << "LRL" << std::endl;
			//std::cout << "centerDistance " << centerDistance << std::endl;
			// direction of Vector(S1,S2) to Vector(S1,S3)
			//double alpha = acos(centerDistance / radius / 4);
			//double alpha = acos(centerDistance / (radius1 * 3 + radius2));

			// direction between first and second arc
			//double dir12 = diff.getAngle() + M_PI / 2 + alpha;
			// direction between second and third arc
			//double dir23 = diff.getAngle() - M_PI / 2 - alpha;

			double alpha = acos(
					(centerDistance * centerDistance + 4 * radius1 * radius1 - sum_radii * sum_radii)
							/ (2 * 2 * radius1 * centerDistance));
			double alpha2 = acos(
					(centerDistance * centerDistance + sum_radii * sum_radii - 4 * radius1 * radius1)
							/ (2 * centerDistance * sum_radii));

			// direction between first and second arc
			double dir12 = diff.getAngle() + M_PI / 2.0 - alpha;
			// direction between second and third arc
			double dir23 = diff.getAngle() - M_PI / 2.0 + alpha2;

			/*
			std::cout << "alpha " << alpha << std::endl;
			std::cout << "alpha2 " << alpha2 << std::endl;

			std::cout << "radius1 = " << radius1 << std::endl;
			std::cout << "radius2 = " << radius2 << std::endl;

			std::cout << "dir12 " << dir12 << std::endl;
			std::cout << "dir23 " << dir23 << std::endl;

			std::cout << "c1left = [" << c1left.x << " , " << c1left.y << "]" << std::endl;
			std::cout << "c2left = [" << c2left.x << " , " << c2left.y << "]" << std::endl;

			std::cout << "diff angle " << diff.getAngle() << std::endl;
			 */
			/*
			 // direction between first and second arc
			 double dir12 = diff.getAngle() - M_PI / 2 + alpha;
			 // direction between second and third arc
			 double dir23 = diff.getAngle() - M_PI / 2 + alpha2;
			 */
			n1 = angleToLeft(from.ang, dir12);
			n2 = angleToRight(dir12, dir23);
			n3 = angleToLeft(dir23, to.ang);

			nLength = radius1 * (fabs(n1) + fabs(n2)) + radius2 * fabs(n3);
			//std::cout << "nLength " << nLength << std::endl;
			if (nLength < length) {
				isCCC = true;
				len1 = n1;
				len2 = n2;
				len3 = n3;
				length = nLength;
				used = true;
				type = DType::LRL;
			}
		}
	}

	if (used) {
		start = from;
		end = to;
	}
}

}
