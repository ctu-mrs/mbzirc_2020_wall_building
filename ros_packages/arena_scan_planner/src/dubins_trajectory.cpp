/*
 * dubins_trajectory.cpp
 *
 *  Created on: Aug 7, 2019
 *      Author: robert
 */

#include "dubins_trajectory.h"

using namespace opendubins;

#define CHECK_SAMPLES false

const double OVER_LIMIT_TOLERANCE = 1e-5;


DubinsTrajectory::DubinsTrajectory(double max_velocity_, double max_acceleration_) {
	// TODO Auto-generated constructor stub
	this->max_velocity = max_velocity_;
	this->max_acceleration = max_acceleration_;
	this->time_sample = 0.2;
}

DubinsTrajectory::~DubinsTrajectory() {
	// TODO Auto-generated destructor stub
}

opendubins::Point DubinsTrajectory::pos_in_distance(opendubins::Point start, opendubins::Point stop, double dist) {
	const double dist_tot = start.distance(stop);

	const double x = start.x + (stop.x - start.x) * dist / dist_tot;
	const double y = start.y + (stop.y - start.y) * dist / dist_tot;

	return opendubins::Point(x, y);
}

SamplesWithTime<opendubins::Point> DubinsTrajectory::sample_euclidean_with_stops(opendubins::Point start, opendubins::Point stop,
		double init_velocity, double final_velocity, double sample_start_time) {

	std::vector<opendubins::Point> samples;
	double trajectory_part_time = 0;

	//ROS_INFO_STREAM("sample_euclidean_with_stops init_velocity "<<init_velocity<<" final_velocity "<<final_velocity)

	double dist_total = start.distance(stop);
	//ROS_INFO_STREAM("sample_euclidean_with_stops dist_total "<< dist_total);

	double time_from_init_to_max_vel = (this->max_velocity - init_velocity) / this->max_acceleration;
	double time_from_max_to_final_vel = (this->max_velocity - final_velocity) / this->max_acceleration;

	double dist_from_init_to_max_vel = 0.5 * (this->max_velocity + init_velocity) * time_from_init_to_max_vel;

	double dist_from_max_vel_to_final = 0.5 * (this->max_velocity + final_velocity) * time_from_max_to_final_vel;

	//ROS_INFO_STREAM("sample_euclidean_with_stops time_from_init_to_max_vel "<< time_from_init_to_max_vel<< "s")
	//ROS_INFO_STREAM("sample_euclidean_with_stops time_from_max_to_final_vel "<< time_from_max_to_final_vel<<"s")
	//ROS_INFO_STREAM("sample_euclidean_with_stops dist_from_init_to_max_vel "<< dist_from_init_to_max_vel<< "m")
	//ROS_INFO_STREAM("sample_euclidean_with_stops dist_from_max_vel_to_final "<< dist_from_max_vel_to_final<< "m")

	double time_to_possible_max_vel = 0;
	double velocity_in_middle = 0;

	if (dist_total < dist_from_init_to_max_vel + dist_from_max_vel_to_final) {
		//can not reach maximal speed in straigh line
		//ROS_INFO_STREAM("can not reach max vel in trajectory")
		double t = 0;
		int sample = 0;

		if (init_velocity == 0 and final_velocity == 0) {
			time_to_possible_max_vel = sqrt(dist_total / this->max_acceleration);
			velocity_in_middle = time_to_possible_max_vel * this->max_acceleration;
			trajectory_part_time = 2 * time_to_possible_max_vel;
		} else {
			if (init_velocity > final_velocity) {
				//initial velocity is larger than final, in the end is additinal decelerating
				const double time_final_decel = (init_velocity - final_velocity) / this->max_acceleration;
				//ROS_INFO_STREAM("sample_euclidean_with_stops1 time_final_decel "<<time_final_decel)
				const double dist_final_decel = time_final_decel * (init_velocity + final_velocity) * 0.5;
				//ROS_INFO_STREAM("sample_euclidean_with_stops1 dist_final_decel "<<dist_final_decel)
				const double dist_acc_decc = dist_total - dist_final_decel;
				//ROS_INFO_STREAM("sample_euclidean_with_stops1 init_velocity "<<init_velocity)
				time_to_possible_max_vel = (-init_velocity + sqrt(init_velocity * init_velocity + this->max_acceleration * dist_acc_decc))
						/ this->max_acceleration;
				//ROS_INFO_STREAM("sample_euclidean_with_stops1 time_to_possible_max_vel "<<time_to_possible_max_vel)
				velocity_in_middle = init_velocity + time_to_possible_max_vel * this->max_acceleration;
				trajectory_part_time = 2 * time_to_possible_max_vel + time_final_decel;
			} else {
				const double time_init_accel = (final_velocity - init_velocity) / this->max_acceleration;
				//ROS_INFO_STREAM("sample_euclidean_with_stops2 time_init_accel "<<time_init_accel)
				const double dist_init_accel = time_init_accel * (init_velocity + final_velocity) * 0.5;
				//ROS_INFO_STREAM("sample_euclidean_with_stops2 dist_init_accel "<<dist_init_accel)
				const double dist_acc_decc = dist_total - dist_init_accel;
				//ROS_INFO_STREAM("sample_euclidean_with_stops2 final_velocity "<<final_velocity)
				time_to_possible_max_vel = time_init_accel
						+ (-final_velocity + sqrt(final_velocity * final_velocity + this->max_acceleration * dist_acc_decc)) / this->max_acceleration;
				//ROS_INFO_STREAM("sample_euclidean_with_stops2 time_to_possible_max_vel "<<time_to_possible_max_vel)
				velocity_in_middle = init_velocity + time_to_possible_max_vel * this->max_acceleration;
				trajectory_part_time = 2 * time_to_possible_max_vel - time_init_accel; // changed time_init_accel from minus to plus
			}
		}

		//INFO_VAR(time_to_possible_max_vel);
		//INFO_VAR(sample_start_time);

		while ((sample + 1) * this->time_sample <= time_to_possible_max_vel - sample_start_time) {
			t = (sample + 1) * this->time_sample + sample_start_time;
			sample += 1;
			const double v = init_velocity + this->max_acceleration * t;
			const double s = init_velocity * t + 0.5 * this->max_acceleration * (t * t);
			opendubins::Point pos_in_dist = pos_in_distance(start, stop, s);
			//ROS_INFO_STREAM("add state "<<pos_in_dist)
			//ROS_INFO_STREAM("t "<< t<< " v "<< v<< " s "<< s<< " sample "<< sample)

			samples.push_back(pos_in_dist);
			if (CHECK_SAMPLES) {
				check_velocity_point_samples(samples);
			}
		}

		//ROS_INFO_STREAM("end acc")

		while ((sample + 1) * this->time_sample <= trajectory_part_time - sample_start_time) {
			t = (sample + 1) * this->time_sample + sample_start_time;
			sample += 1;
			const double t_part = t - time_to_possible_max_vel;
			const double v = velocity_in_middle - this->max_acceleration * t_part;
			const double s = time_to_possible_max_vel * 0.5 * (velocity_in_middle + init_velocity) + velocity_in_middle * t_part
					- 0.5 * this->max_acceleration * (t_part * t_part);
			opendubins::Point pos_in_dist = pos_in_distance(start, stop, s);
			//ROS_INFO_STREAM("add state "<<pos_in_dist)
			//ROS_INFO_STREAM("t "<< t<< " v "<< v<< " s "<< s<< " sample "<< sample)

			samples.push_back(pos_in_dist);
			if (CHECK_SAMPLES) {
				check_velocity_point_samples(samples);
			}
		}

		//ROS_INFO_STREAM("end decc")

	} else {
		// can reach maximal speed in straigh line
		//ROS_INFO_STREAM("can reach max vel")
		double dist_constant_speed = dist_total - dist_from_init_to_max_vel - dist_from_max_vel_to_final;
		double time_constant_speed = dist_constant_speed / this->max_velocity;
		trajectory_part_time = time_from_init_to_max_vel + time_constant_speed + time_from_max_to_final_vel;

		//ROS_INFO_STREAM("time_constant_speed "<< time_constant_speed<< "s");
		//ROS_INFO_STREAM("dist_constant_speed "<< dist_constant_speed<< "m");
		//ROS_INFO_STREAM("trajectory_part_time "<<trajectory_part_time);

		double t = 0;
		int sample = 0;

		while ((sample + 1) * this->time_sample <= time_from_init_to_max_vel - sample_start_time) {
			t = (sample + 1) * this->time_sample + sample_start_time;
			sample += 1;
			const double v = init_velocity + this->max_acceleration * t;
			const double s = init_velocity * t + 0.5 * this->max_acceleration * (t * t);
			opendubins::Point pos_in_dist = pos_in_distance(start, stop, s);
			//ROS_INFO_STREAM("add state "<<pos_in_dist)
			samples.push_back(pos_in_dist);
			//ROS_INFO_STREAM("t "<< t<< " v "<< v<< " s "<< s<< " sample "<< sample)
			if (CHECK_SAMPLES) {
				check_velocity_point_samples(samples);
			}
		}

		//ROS_INFO_STREAM("end acc")

		while ((sample + 1) * this->time_sample <= time_from_init_to_max_vel + time_constant_speed - sample_start_time) {
			t = (sample + 1) * this->time_sample + sample_start_time;
			sample += 1;
			const double t_part = t - time_from_init_to_max_vel;
			const double v = this->max_velocity;
			const double s = dist_from_init_to_max_vel + v * t_part;
			opendubins::Point pos_in_dist = pos_in_distance(start, stop, s);
			//ROS_INFO_STREAM("add state "<<pos_in_dist)
			samples.push_back(pos_in_dist);
			//ROS_INFO_STREAM("t "<< t<< " v "<< v<< " s "<< s<< " sample "<< sample)
			if (CHECK_SAMPLES) {
				check_velocity_point_samples(samples);
			}
		}

		//ROS_INFO_STREAM("end const")

		while ((sample + 1) * this->time_sample <= time_from_init_to_max_vel + time_constant_speed + time_from_max_to_final_vel - sample_start_time) {
			t = (sample + 1) * this->time_sample + sample_start_time;
			sample += 1;
			const double t_part = t - (time_from_init_to_max_vel + time_constant_speed);
			const double v = this->max_velocity - this->max_acceleration * t_part;
			const double s = (dist_total - dist_from_max_vel_to_final) + this->max_velocity * t_part
					- 0.5 * this->max_acceleration * (t_part * t_part);
			opendubins::Point pos_in_dist = pos_in_distance(start, stop, s);
			//ROS_INFO_STREAM("add state "<<pos_in_dist)

			//ROS_INFO_STREAM("t "<< t<< " v "<< v<< " s "<< s<< " sample "<<sample);

			samples.push_back(pos_in_dist);
			if (CHECK_SAMPLES) {
				check_velocity_point_samples(samples);
			}
		}

		//ROS_INFO_STREAM("end decc")
	}

	if (final_velocity == 0 && (samples[samples.size() - 1].x != stop.x || samples[samples.size() - 1].y != stop.y)) {
		//ROS_INFO_STREAM("t last "<< " v " << 0<< " s "<< dist_total);
		//ROS_INFO_STREAM("add state "<<stop)
		samples.push_back(stop);
	}
	SamplesWithTime<opendubins::Point> to_return;
	to_return.samples = samples;
	to_return.flight_time = trajectory_part_time;
	return to_return;

}

SamplesWithTime<opendubins::Point> DubinsTrajectory::sample_trajectory_euclidean(std::vector<opendubins::Point> sequence) {

	// sample euclidean tarjectory over sequence

	ROS_INFO_STREAM("sample_trajectory_euclidean in sequence" << sequence.size());

	std::vector<opendubins::Point> samples;
	samples.push_back(sequence[0]);
	// add first point of trajectory
	double trajectory_time = 0;

	for (int target_id = 0; target_id < sequence.size(); ++target_id) {

		const opendubins::Point &from_target = sequence[target_id - 1];
		const opendubins::Point &to_target = sequence[target_id];
		SamplesWithTime<opendubins::Point> part = this->sample_euclidean_with_stops(from_target, to_target);
		trajectory_time += part.flight_time;
		ROS_INFO_STREAM("part_time "<< part.flight_time);
		samples.insert(samples.end(), part.samples.begin(), part.samples.end());
		//return samples, trajectory_time
	}
	SamplesWithTime<opendubins::Point> to_return;
	to_return.samples = samples;
	to_return.flight_time = trajectory_time;
	return to_return;
}

SamplesWithTime<opendubins::State> DubinsTrajectory::sample_trajectory_dubins(std::vector<GraphNode_AngNeigh> sequence) {
	/// sample dubins tarjectory over sequence

	//double turning_radius = (turning_velocity * turning_velocity) / this->max_acceleration;
	//ROS_INFO_STREAM("which means turning_radius" << turning_radius);

	//double sequence_start = 0;

	double init_velocity = 0;
	init_velocity = sqrt(sequence[0].radius * this->max_acceleration);
	ROS_INFO_STREAM("using init_velocity "<<init_velocity);

	double time_to_turning_velocity = 0;
	if (init_velocity == 0) {
		double first_turning_velocity = velocity_of_radius(sequence[0].radius, this->max_acceleration);
		time_to_turning_velocity = (first_turning_velocity) / this->max_acceleration;
		double dist_to_turning_velocity = 0.5 * (first_turning_velocity) * time_to_turning_velocity; // average speed * time
	}
	//INFO_VAR(time_to_turning_velocity)
	//INFO_VAR(dist_to_turning_velocity)

	std::vector<opendubins::State> samples;
	int sample = 0;
	double t = 0;
	double s = 0;
	double last_segment_end_time = 0;
	double next_sample_start_time = 0;
	samples.push_back(sequence[0].toState());
	//ROS_INFO_STREAM("t", t, "v", 0, "s", 0, "sample", sample, "start")

	for (int target_id = 1; target_id < sequence.size(); ++target_id) {

		GraphNode_AngNeigh start = sequence[target_id - 1];
		GraphNode_AngNeigh end = sequence[target_id];
		//INFO_VAR(start.toState().point)
		//INFO_VAR(end.toState().point)
		//ROS_INFO_STREAM("part "<<target_id<<" from radius "<< start.radius<<" to radius "<< end.radius)

		opendubins::Dubins dubins_path = opendubins::Dubins(start.toState(), end.toState(), start.radius, end.radius);

		//ROS_INFO_STREAM("dubins type "<<dubins_path.getType());
		//ROS_INFO_STREAM("segment 0 "<< dubins_path.getLen1Dist())
		//ROS_INFO_STREAM("segment 1 "<< dubins_path.getLen2Dist())
		//ROS_INFO_STREAM("segment 2 "<< dubins_path.getLen3Dist())
		//ROS_INFO_STREAM("dubins lenght "<< dubins_path.length)

		//INFO_VAR(start.radius)
		//INFO_VAR(end.radius)

		double turning_velocity_segment_1 = velocity_of_radius(start.radius, this->max_acceleration);
		double turning_velocity_segment_2 = velocity_of_radius(end.radius, this->max_acceleration);

		double segment_1_time;
		// first segment of dubins

		if ((sample + 1) * this->time_sample < time_to_turning_velocity) {
			init_velocity = last_segment_end_time * this->max_acceleration;
			const double time_accel = (turning_velocity_segment_1 - init_velocity) / this->max_acceleration;
			const double dist_accel = init_velocity * time_accel + 0.5 * this->max_acceleration * time_accel * time_accel;

			//INFO_VAR(time_accel)
			//INFO_VAR(dist_accel)

			if (dubins_path.getLen1Dist() < dist_accel) {		// accel whole time
				segment_1_time = (-init_velocity + sqrt(init_velocity * init_velocity + 2 * this->max_acceleration * dubins_path.getLen1Dist()))
						/ this->max_acceleration;		// turning segment 0
			} else {		// accel only part time
				segment_1_time = time_accel + (dubins_path.getLen1Dist() - dist_accel) / turning_velocity_segment_1;
			}
		} else {
			segment_1_time = dubins_path.getLen1Dist() / turning_velocity_segment_1;		// turning segment 0
			init_velocity = turning_velocity_segment_1;
		}

		//ROS_INFO_STREAM("last_segment_end_time", last_segment_end_time)
		//ROS_INFO_STREAM("segment_1_time",segment_1_time)
		//INFO_VAR(segment_1_time)

		/*
		 const double acc_time = turning_velocity / this->max_acceleration;
		 const double segment_1_time_dist = 0.5 * this->max_acceleration * acc_time * acc_time
		 + (segment_1_time - acc_time) * turning_velocity;
		 */
		while ((sample + 1) * this->time_sample <= last_segment_end_time + segment_1_time) {
			t = (sample + 1) * this->time_sample - last_segment_end_time;

			if (init_velocity != turning_velocity_segment_1) {
				if ((sample + 1) * this->time_sample <= time_to_turning_velocity) {	// still accelerating from init_velocity
					s = init_velocity * t + 0.5 * this->max_acceleration * (t * t);
				} else {
					const double dist_init_acc = 0.5 * (turning_velocity_segment_1 + init_velocity) * time_to_turning_velocity;	// alreaddy accelerated from init_velocity to turning_velocity

					const double time_after_init_acc = t - time_to_turning_velocity;
					s = dist_init_acc + turning_velocity_segment_1 * time_after_init_acc;
				}
			} else {	// already turning velocity from begining
				s = turning_velocity_segment_1 * t;
			}
			sample += 1;
			State state_to_add = dubins_path.getState(s);
			//ROS_INFO_STREAM("add state "<<state_to_add.point)
			samples.push_back(state_to_add);
			if (CHECK_SAMPLES) {
				check_velocity_state_samples(samples);
			}
			//ROS_INFO_STREAM(					"t "<< t<< " s "<< s<< " sample "<< sample<<" sample time "<< sample*this->time_sample<< " dubins length "<< dubins_path.length<< " dubins part len "<< dubins_path.getLen1Dist()<< " rot1")
		}

		last_segment_end_time += segment_1_time;
		next_sample_start_time = sample * this->time_sample - last_segment_end_time;

		if (next_sample_start_time > this->time_sample) {
			ROS_INFO_STREAM("can not have such a large next_sample_start_time "<<next_sample_start_time);
			exit(1);
		}

		if (last_segment_end_time < time_to_turning_velocity) {
			init_velocity = last_segment_end_time * this->max_acceleration;
		} else {
			init_velocity = turning_velocity_segment_1;
		}

		//ROS_INFO_STREAM("---------- end fist segment --------------- at time "<< last_segment_end_time);

		// second segment of Dubins
		double segment_2_time = 0;
		if (!dubins_path.isCCC) {	// straight line segment
			opendubins::State start_straight_line = dubins_path.getState(dubins_path.getLen1Dist());
			opendubins::State stop_straight_line = dubins_path.getState(dubins_path.getLen1Dist() + dubins_path.getLen2Dist());
			//ROS_INFO_STREAM("straight line segment");
			//ROS_INFO_STREAM("start_straight_line "<< start_straight_line)
			//ROS_INFO_STREAM("stop_straight_line "<< stop_straight_line)
			//ROS_INFO_STREAM("init_velocity "<< init_velocity)
			//ROS_INFO_STREAM("final_velocity "<< turning_velocity)
			//ROS_INFO_STREAM("next_sample_start_time "<< next_sample_start_time)
			//double turning_velocity_segment_1 = velocity_of_radius(start.radius, this->max_acceleration);
			SamplesWithTime<opendubins::Point> straight_segment = this->sample_euclidean_with_stops(start_straight_line.point,
					stop_straight_line.point, init_velocity, turning_velocity_segment_2, next_sample_start_time);
			const double phi = start_straight_line.ang;
			segment_2_time = straight_segment.flight_time;
			for (int var = 0; var < straight_segment.samples.size(); ++var) {
				State new_state(straight_segment.samples[var], phi);
				samples.push_back(new_state);
			}
			sample += straight_segment.samples.size();
		} else { // also circular segment
			//ROS_INFO_STREAM("CCC part");
			//segment_2_time = dubins_path.getLen2Dist() / turning_velocity_segment_1; // turning segment 1
			double dist_len2 = dubins_path.getLen2Dist();
			//sample distance to x variable
			SamplesWithTime<opendubins::Point> straight_segment = this->sample_euclidean_with_stops(Point(0, 0), Point(dist_len2, 0), init_velocity,
					turning_velocity_segment_2, next_sample_start_time);
			segment_2_time = straight_segment.flight_time;

			for (int var = 0; var < straight_segment.samples.size(); ++var) {
				t = (sample + 1) * this->time_sample - last_segment_end_time;
				sample += 1;
				s = dubins_path.getLen1Dist() + straight_segment.samples[var].x;
				State state_to_add = dubins_path.getState(s);
				samples.push_back(state_to_add);
				if (CHECK_SAMPLES) {
					check_velocity_state_samples(samples);
				}
				//ROS_INFO_STREAM(						"t "<< t<< " s "<< s<< " sample "<< sample<< " dubins length "<< dubins_path.length<< " rot middle")
			}
			/*
			 while ((sample + 1) * this->time_sample <= last_segment_end_time + segment_2_time) {
			 t = (sample + 1) * this->time_sample - last_segment_end_time;
			 sample += 1;
			 // t_part = t - last_segment_end_time
			 s = dubins_path.getLen1Dist() + turning_velocity_segment_1 * t;
			 State state_to_add = dubins_path.getState(s);
			 //ROS_INFO_STREAM("add state "<<state_to_add.point)
			 samples.push_back(state_to_add);
			 if (CHECK_SAMPLES) {
			 check_velocity_state_samples(samples);
			 }
			 //ROS_INFO_STREAM(						"t "<< t<< " s "<< s<< " sample "<< sample<< " dubins length "<< dubins_path.length<< " rot middle")
			 }
			 */
		}
		last_segment_end_time += segment_2_time;
		next_sample_start_time = sample * this->time_sample - last_segment_end_time;

		if (next_sample_start_time > this->time_sample) {
			ROS_INFO_STREAM("can not have such a large next_sample_start_time "<<next_sample_start_time);
			exit(1);
		}

		if ((sample + 1) * this->time_sample < time_to_turning_velocity) {
			init_velocity = last_segment_end_time * this->max_acceleration;
		} else {
			init_velocity = turning_velocity_segment_2;
		}

		//ROS_INFO_STREAM("---------- end second segment --------------- at time" << last_segment_end_time)

		double segment_3_time = dubins_path.getLen3Dist() / turning_velocity_segment_2; // turning segment 2
		while ((sample + 1) * this->time_sample <= last_segment_end_time + segment_3_time) {
			t = (sample + 1) * this->time_sample - last_segment_end_time;
			sample += 1;
			if (init_velocity != turning_velocity_segment_2) {
				s = dubins_path.getLen1Dist() + dubins_path.getLen2Dist() + init_velocity * t + 0.5 * this->max_acceleration * (t * t);
			} else {
				s = dubins_path.getLen1Dist() + dubins_path.getLen2Dist() + turning_velocity_segment_2 * t;
			}
			State state_to_add = dubins_path.getState(s);
			//ROS_INFO_STREAM("add state "<<state_to_add.point)
			samples.push_back(state_to_add);
			if (CHECK_SAMPLES) {
				check_velocity_state_samples(samples);
			}
			//ROS_INFO_STREAM("t "<< t<< " v "<< turning_velocity<< " s "<< s<< " sample "<< sample<< " rot2")
		}

		last_segment_end_time += segment_3_time;
		next_sample_start_time = sample * this->time_sample - last_segment_end_time;

		if (next_sample_start_time > this->time_sample) {
			ROS_INFO_STREAM("can not have such a large next_sample_start_time "<<next_sample_start_time);
			exit(1);
		}

		if (last_segment_end_time < time_to_turning_velocity) {
			init_velocity = last_segment_end_time * this->max_acceleration;
		} else {
			init_velocity = turning_velocity_segment_2;
		}

//ROS_INFO_STREAM("---------- end last segment --------------- at time "<< last_segment_end_time)
//break;

		//ROS_INFO_STREAM(	"part "<<target_id<<" from "<<start.node.x<<" "<<start.node.y<<" "<<start.ang<<" to "<<end.node.x <<" "<<end.node.y <<" "<<end.ang<<" time length "<<(segment_1_time + segment_2_time + segment_3_time))

		//ROS_INFO_STREAM("part "<<target_id<<" times "<<segment_1_time<<" "<<segment_2_time<<" "<<segment_3_time)
		//ROS_INFO_STREAM("dubins_path.radius1 "<<dubins_path.radius1<<" dubins_path.radius2 "<<dubins_path.radius2)

		const double dub_time_len = dubins_path.getTimeLength(max_velocity, max_acceleration);
		const double here_calculated_time_len = (segment_1_time + segment_2_time + segment_3_time);
		if (fabs(dub_time_len - here_calculated_time_len) > 0.01) {
			ROS_ERROR_STREAM("probably badly calculated dubins time length");
			ROS_ERROR_STREAM("dub_time_len "<<dub_time_len);
			ROS_ERROR_STREAM("here_calculated_time_len "<<here_calculated_time_len);
			exit(0);
		}

	}

	if (CHECK_SAMPLES) {
		check_velocity_state_samples(samples);
	}
	SamplesWithTime<opendubins::State> to_return;
	to_return.samples = samples;
	to_return.flight_time = last_segment_end_time;
	return to_return;
}

void DubinsTrajectory::check_velocity_point_samples(std::vector<opendubins::Point> &samples) {
	std::vector<double> velocities;
	velocities.reserve(samples.size());
	//velocities.push_back(0);

	//check velocity
	for (int var = 1; var < samples.size(); ++var) {
		double vel = samples[var - 1].distance(samples[var]) / this->time_sample;
		if (var == 1) {
			velocities.push_back(vel);
		}
		if (vel > this->max_velocity + OVER_LIMIT_TOLERANCE) {
			ROS_INFO_STREAM("max velocity exceeded in check_velocity_point_samples");
			std::cout.precision(10);
			ROS_INFO_STREAM("this->max_velocity = "<<this->max_velocity);
			ROS_INFO_STREAM("vel = "<<vel);

			exit(1);
		}
		velocities.push_back(vel);
	}
	check_aceleration(velocities);
}

void DubinsTrajectory::check_velocity_state_samples(std::vector<opendubins::State> &samples) {
	std::vector<double> velocities;
	velocities.reserve(samples.size());
	//velocities.push_back(0);

	//check velocity
	for (int var = 1; var < samples.size(); ++var) {
		double vel = samples[var - 1].point.distance(samples[var].point) / this->time_sample;
		if (var == 1) {
			velocities.push_back(vel);
		}
		if (vel > this->max_velocity + OVER_LIMIT_TOLERANCE) {
			ROS_INFO_STREAM("max velocity exceeded in check_velocity_state_samples");
			ROS_INFO_STREAM("this->max_velocity = "<<this->max_velocity);
			ROS_INFO_STREAM("vel = "<<vel);
			exit(1);
		}
		velocities.push_back(vel);
	}
	check_aceleration(velocities);
}

void DubinsTrajectory::check_aceleration(std::vector<double> &velocities) {
	std::vector<double> accelerations;
	accelerations.reserve(velocities.size());
	accelerations.push_back(0);

//check accelerations
	for (int var = 1; var < velocities.size(); ++var) {
		double acc = fabs(velocities[var - 1] - velocities[var]) / this->time_sample;
		if (acc > this->max_acceleration + OVER_LIMIT_TOLERANCE) {
			ROS_INFO_STREAM("max acceleration exceeded");
			ROS_INFO_STREAM("this->max_acceleration = "<<this->max_acceleration);
			ROS_INFO_STREAM("acc = "<<acc);
			exit(1);
		}
		accelerations.push_back(acc);
	}
}
