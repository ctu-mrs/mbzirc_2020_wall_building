/*
 * dubins_trajectory.h
 *
 *  Created on: Aug 7, 2019
 *      Author: robert
 */

#ifndef DUBINS_TRAJECTORY_H_
#define DUBINS_TRAJECTORY_H_

//#include "crl/logging.h"
#include "dubins/dubins.h"
#include <ros/ros.h>

using namespace opendubins;

//#include "heuristic_types.h"

/*
typedef struct GraphNode {

	GraphNode() {
		x = 0;
		y = 0;
		reward = 0;
		id = 0;
	}

	GraphNode(double x_, double y_, double price_, unsigned int id_) {
		x = x_;
		y = y_;
		reward = price_;
		id = id_;
	}
	std::vector<double> toVector() {
		std::vector<double> vector;
		vector.push_back(x);
		vector.push_back(y);
		vector.push_back(reward);
		return vector;
	}
	Point toPoint() {
		return Point(this->x, this->y);
	}
	double distanceTo(GraphNode gn1) const {
		double diffx = gn1.x - this->x;
		double diffy = gn1.y - this->y;
		return sqrt(diffx * diffx + diffy * diffy);
	}

	double x;
	double y;
	double reward;
	unsigned int id;
} GraphNode;
*/

typedef struct GraphNode_AngNeigh {
	GraphNode_AngNeigh() {
		node = Point();
		ang = 0;
		neigh_ang = 0;
		radius = 0;
	}

	GraphNode_AngNeigh(Point node_, double ang_, double neigh_ang_, double radius_) {
		node = node_;
		ang = ang_;
		neigh_ang = neigh_ang_;
		radius = radius_;
	}

	State toState() {
		return State(node, ang);
	}

	Point node;
	double ang;
	double neigh_ang;
	double radius;
} GraphNode_AngNeigh;

template<typename T>
struct SamplesWithTime {
	std::vector<T> samples;
	double flight_time;
};

//inline double radius_of_velocity(double velocity, double max_acceleration);
//inline double velocity_of_radius(double radius, double max_acceleration);

inline double radius_of_velocity(double velocity, double max_acceleration) {
	return (velocity * velocity) / max_acceleration;		//this->max_acceleration;
}

inline double velocity_of_radius(double radius, double max_acceleration) {
	return sqrt(radius * max_acceleration);		//this->max_acceleration);
}


class DubinsTrajectory {
public:
	DubinsTrajectory(double max_velocity_, double max_acceleration_);
	virtual ~DubinsTrajectory();
	static opendubins::Point pos_in_distance(opendubins::Point start, opendubins::Point stop, double dist);
	SamplesWithTime<opendubins::Point> sample_euclidean_with_stops(opendubins::Point start, opendubins::Point stop, double init_velocity = 0,
			double final_velocity = 0, double sample_start_time = 0);
	SamplesWithTime<opendubins::Point> sample_trajectory_euclidean(std::vector<opendubins::Point> sequence);
	SamplesWithTime<opendubins::State> sample_trajectory_dubins(std::vector<GraphNode_AngNeigh> sequence);

private:
	double max_velocity;
	double max_acceleration;
	double time_sample;

	void check_velocity_point_samples(std::vector<opendubins::Point> & samples);
	void check_velocity_state_samples(std::vector<opendubins::State> & samples);
	void check_aceleration(std::vector<double> & velocities);
};

#endif /* DUBINS_TRAJECTORY_H_ */
