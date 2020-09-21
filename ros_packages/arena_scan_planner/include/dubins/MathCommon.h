/**
 * MathCommon.h
 *
 *  Created on: 16. 11. 2014
 *    @au Robert Pěnička
 */

#ifndef MATHCOMMON_H_
#define MATHCOMMON_H_
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <float.h>
#include <vector>

#ifndef PI_
#define PI_ 3.1415926535897932385
#endif

#define M_2PI (2 * PI_)
#define POW(x) ((x)*(x))
#define MIN(x,y) ((x > y) ? y : x)
#define MAX(x,y) ((x > y) ? ( x ) : ( y ))
#define ABS(x) ((x < 0) ? (-(x)) : ( x ))
#define ANGLE_MIN (0)
#define ANGLE_MAX (M_2PI)

double randDoubleMinMax(double min, double max);
int randIntMinMax(int min, int max);
/**
 * Normalizes angle in range <0,2pi].
 * @param angle
 * @return angle in range <0,2pi]
 */
double normalizeAngle(double angle,double min=ANGLE_MIN,double max=ANGLE_MAX);
std::vector<double> range(double min ,double max,double step = 1.0);

int sgn(int val);
double sgn(double val);
float sgn(float val);

#endif /* MATHCOMMON_H_ */
