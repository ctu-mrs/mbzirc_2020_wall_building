/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <unordered_set>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <limits>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

#include <arena_scan_planner/ArenaScanTrajectory.h>
//#include <mbzirc_trackers/Reference.h>

//#include <mbzirc_estimation/convex_polygon.h>
#include <eigen3/Eigen/Eigen>

//#include <sweeping_planner/CSort.h>
#include <dubins/MathCommon.h>
#include <dubins/state.h>
#include <dubins/dubins.h>

#include <dubins_trajectory.h>

#include <vector>

using namespace opendubins;
using namespace Eigen;

#define OUTPUT_DEFAULT "\033[0m"
#define OUTPUT_BLACK "\033[30m"
#define OUTPUT_RED "\033[31m"
#define OUTPUT_GREEN "\033[32m"
#define OUTPUT_YELLOW "\033[33m"
#define OUTPUT_BLUE "\033[34m"
#define OUTPUT_MAGENTA "\033[35m"
#define OUTPUT_CYAN "\033[36m"
#define OUTPUT_WHITE "\033[37m"

#define ROS_INFO_STREAM_RED(x) ROS_INFO_STREAM(OUTPUT_RED << x << OUTPUT_DEFAULT)
#define ROS_INFO_STREAM_YELLOW(x) ROS_INFO_STREAM(OUTPUT_YELLOW << x << OUTPUT_DEFAULT)
#define ROS_INFO_STREAM_MAGENTA(x) ROS_INFO_STREAM(OUTPUT_MAGENTA << x << OUTPUT_DEFAULT)
#define ROS_INFO_STREAM_CYAN(x) ROS_INFO_STREAM(OUTPUT_CYAN << x << OUTPUT_DEFAULT)
#define ROS_INFO_STREAM_GREEN(x) ROS_INFO_STREAM(OUTPUT_GREEN << x << OUTPUT_DEFAULT)
#define ROS_INFO_STREAM_WHITE(x) ROS_INFO_STREAM(OUTPUT_WHITE << x << OUTPUT_DEFAULT)
#define ROS_INFO_STREAM_BLUE(x) ROS_INFO_STREAM(OUTPUT_BLUE << x << OUTPUT_DEFAULT)
#define ROS_INFO_STREAM_BLACK(x) ROS_INFO_STREAM(OUTPUT_BLACK << x << OUTPUT_DEFAULT)

class ArenaScanPlanner {
private:
  ros::NodeHandle    nh_;
  ros::ServiceServer sweep_plan_service_;

  MatrixXd arenaCorners;

  double fieldofview_deg;
  double overlap;
  double flight_yaw;
  double mapping_ground_altitude;
  double skip_arena_sides_displacement;
  // double radius_rate_shift;
  // double radius_rate_shift_first_last;
  // std::vector<Point> home_positions;

public:
  ArenaScanPlanner(MatrixXd arenaCorners_, double fieldofview_deg_, double overlap_, double radius_rate_shift_, double radius_rate_shift_first_last_,
                   double flight_yaw_, double mapping_ground_altitude_, double skip_arena_sides_displacement_);
  ~ArenaScanPlanner();
  bool scan_plan_callback(arena_scan_planner::ArenaScanTrajectory::Request &req, arena_scan_planner::ArenaScanTrajectory::Response &res);
  std::vector<std::vector<State>> getArenaScanPath(double sweepSpeed, double maxSweepAcc, double turningSpeed, double sweep_altitude, double initial_speed,
                                                   bool loop, int num_robots);
  void                            savePathToFile(std::vector<State> &toSave, std::string filename);
};

Point getPointInDistanceInVectorDirection(Vector vector, double dist) {
  double length = vector.length();
  return Point(dist * (vector.dx / length), dist * (vector.dy / length));
}

Point getPointInDistFromVector(Point fromPoint, double distance, Vector vector) {
  double vectorLength = vector.length();
  return Point(fromPoint.x + distance * vector.dx / vectorLength, fromPoint.y + distance * vector.dy / vectorLength);
}

Point getPointInDistance(Point point, Line line, double distance) {
  // ROS_INFO_STREAM("getPointInDistance");
  // ROS_INFO_STREAM("distance "<<distance);
  // ROS_INFO_STREAM("lineSegment "<<lineSegment);
  Vector lineSegmentVector = line.getDirection();
  // ROS_INFO_STREAM("lineSegmentVector "<<lineSegmentVector);
  Point pointInDistance = getPointInDistFromVector(point, distance, lineSegmentVector);
  getPointInDistanceInVectorDirection(lineSegmentVector, distance);
  // ROS_INFO_STREAM("pointInDistance "<<pointInDistance);
  return pointInDistance;
}

ArenaScanPlanner::ArenaScanPlanner(MatrixXd arenaCorners_, double fieldofview_deg_, double overlap_, double radius_rate_shift_,
                                   double radius_rate_shift_first_last_, double flight_yaw_, double mapping_ground_altitude_,
                                   double skip_arena_sides_displacement_)
    : nh_("~") {

  this->sweep_plan_service_           = this->nh_.advertiseService("plan", &ArenaScanPlanner::scan_plan_callback, this);
  this->arenaCorners                  = arenaCorners_;
  this->fieldofview_deg               = fieldofview_deg_;
  this->overlap                       = overlap_;
  this->flight_yaw                    = flight_yaw_;
  this->mapping_ground_altitude       = mapping_ground_altitude_;
  this->skip_arena_sides_displacement = skip_arena_sides_displacement_;
  // this->radius_rate_shift = radius_rate_shift_;
  // this->radius_rate_shift_first_last = radius_rate_shift_first_last_;
  // this->home_positions = home_positions_;
}

ArenaScanPlanner::~ArenaScanPlanner() {
}

bool ArenaScanPlanner::scan_plan_callback(arena_scan_planner::ArenaScanTrajectory::Request &req, arena_scan_planner::ArenaScanTrajectory::Response &res) {
  ROS_INFO_STREAM("-------------------------------------------------");
  double sweep_altitude          = req.altitude;
  double sweep_speed             = req.speed;
  double sweep_acc               = req.acc;
  double turning_speed           = req.turning_speed;
  double initial_speed           = req.initial_speed;
  int    requested_arena_part_id = req.arena_part_id - 1;
  int    num_robots              = req.num_robots;
  bool   loop                    = req.loop;
  ROS_INFO_STREAM_GREEN("request robot id " << requested_arena_part_id);
  std::vector<std::vector<State>> sampled_paths = getArenaScanPath(sweep_speed, sweep_acc, turning_speed, sweep_altitude, initial_speed, loop, num_robots);

  if (requested_arena_part_id < 0 || requested_arena_part_id >= sampled_paths.size()) {
    res.ok      = false;
    res.message = "failed to create scanning plan";
  } else {
    res.ok = true;
    std::vector<mrs_msgs::Reference> poseArray;

    double first_yaw = sampled_paths[requested_arena_part_id][0].ang;
    for (int var = 0; var < sampled_paths[requested_arena_part_id].size(); ++var) {
      mrs_msgs::Reference pose;
      State               state = sampled_paths[requested_arena_part_id][var];
      pose.position.x           = state.point.x;
      pose.position.y           = state.point.y;
      pose.position.z           = sweep_altitude;
      pose.heading              = first_yaw + this->flight_yaw;
      // ROS_INFO_STREAM("pose.yaw "<<pose.yaw);
      poseArray.push_back(pose);
    }
    ROS_INFO_STREAM_GREEN("sending path for robot id " << requested_arena_part_id);
    res.sweep_path = poseArray;
    res.message    = "created scanning plan";
  }
  ROS_INFO_STREAM("-------------------------------------------------");
  return true;
}

std::vector<std::vector<State>> ArenaScanPlanner::getArenaScanPath(double maxScanningSpeed, double maxScanningAcc, double turningSpeed,
                                                                   double scanning_altitude, double initial_speed, bool loop, int num_robots) {
  ROS_INFO_STREAM("generating arena scan path with:");
  ROS_INFO_STREAM("using maxScanningSpeed " << maxScanningSpeed);
  ROS_INFO_STREAM("using maxScanningAcc " << maxScanningAcc);
  ROS_INFO_STREAM("using scanning_altitude " << scanning_altitude);
  ROS_INFO_STREAM("using initial_speed " << initial_speed);
  ROS_INFO_STREAM("using loop " << loop);

  bool   flight_relative_to_velocity_dir = false;
  double time_sample                     = 0.2;

  double use_same_initial_yaw = true;

  double initialSpeed = initial_speed;
  double radius       = (turningSpeed * turningSpeed) / maxScanningAcc;

  // double radius = min(distance_from_size);
  ROS_INFO_STREAM("using radius " << radius);

  std::vector<std::vector<State>> sampledPositions(num_robots);
  double                          sampleDistance = maxScanningSpeed * time_sample;

  ROS_INFO_STREAM("arena corners are:");
  ROS_INFO_STREAM(arenaCorners(0, 0) << " " << arenaCorners(0, 1));
  ROS_INFO_STREAM(arenaCorners(1, 0) << " " << arenaCorners(1, 1));
  ROS_INFO_STREAM(arenaCorners(2, 0) << " " << arenaCorners(2, 1));
  ROS_INFO_STREAM(arenaCorners(3, 0) << " " << arenaCorners(3, 1));

  Point K(arenaCorners(0, 0), arenaCorners(0, 1));
  Point L(arenaCorners(1, 0), arenaCorners(1, 1));
  Point M(arenaCorners(2, 0), arenaCorners(2, 1));
  Point N(arenaCorners(3, 0), arenaCorners(3, 1));
  // std::vector<Line> lines;

  // lines.push_back(Line(K, L)); //K
  // lines.push_back(Line(L, M)); //L
  // lines.push_back(Line(M, N)); //M
  // lines.push_back(Line(N, K)); //N

  //    N                                  M
  //     ----------------------------------
  //     |                                |
  //     |               1                |
  //     |--------------------------------|
  //     |                                |
  //     |               2                |
  //     |--------------------------------|
  //     |                                |
  //     |               3                |
  //     ----------------------------------
  //    K                                  L

  // Point basePoint = K;
  // Line baseLineForSweep(K, L);
  // Line sweepAlongLine(K, N);

  Point basePoint = K;
  Line  baseLineForSweep(K, N);
  Line  sweepAlongLine(K, L);

  if (baseLineForSweep.getLength() > sweepAlongLine.getLength()) {
    ROS_WARN_STREAM("baseLineForSweep is longer than sweepAlongLine, switching them");
    Line tmp         = baseLineForSweep;
    baseLineForSweep = sweepAlongLine;
    sweepAlongLine   = tmp;
  }

  double arena_width = baseLineForSweep.getLength() - 2 * skip_arena_sides_displacement;
  ROS_INFO_STREAM("arena_width " << arena_width);
  double fieldofview = (fieldofview_deg / 180.0) * (M_2PI / 2.0);
  ROS_INFO_STREAM("fieldofview " << fieldofview);

  ROS_INFO_STREAM("mapping_ground_altitude " << mapping_ground_altitude);
  double width_on_ground_half = (tan(fieldofview / 2.0) * (scanning_altitude - mapping_ground_altitude));
  double width_on_ground      = width_on_ground_half * 2;
  ROS_INFO_STREAM("width_on_ground " << width_on_ground);

  double width_on_ground_w_overlap = (1.0 - overlap) * width_on_ground;
  ROS_INFO_STREAM("width_on_ground_w_overlap " << width_on_ground_w_overlap);
  double num_sweeps = ceil(arena_width / width_on_ground_w_overlap);


  ROS_INFO_STREAM("number of required sweeps " << num_sweeps);
  double num_sweeps_ideal = ceil(num_sweeps / ((double)num_robots)) * ((double)num_robots);
  ROS_INFO_STREAM("num_sweeps_ideal " << num_sweeps_ideal);
  double num_sweeps_per_robot = num_sweeps_ideal / ((double)num_robots);
  ROS_INFO_STREAM("num_sweeps_per_robot " << num_sweeps_per_robot);

  double width_sweep = arena_width / num_sweeps_ideal;
  ROS_INFO_STREAM("width_sweep " << width_sweep);

  if (radius > (width_sweep / 2.0) - 0.1) {
    ROS_WARN_STREAM("radius " << radius << " is bigger than ((width_sweep / 2.0) - 0.1) " << ((width_sweep / 2.0) - 0.1));
    radius = (width_sweep / 2.0) - 0.1;
    ROS_WARN_STREAM("decreasing radius to " << radius);
  }


  // std::vector<std::vector<State>> robotSweepPoints(num_robots);
  std::vector<std::vector<GraphNode_AngNeigh>> arenaBorderScanPoints(num_robots);

  for (int robotID = 0; robotID < num_robots; ++robotID) {
    ROS_INFO_STREAM_GREEN("robotID " << robotID);
    double distanceInVector = num_sweeps_per_robot * robotID * width_sweep + width_sweep / 2.0 + skip_arena_sides_displacement;
    // ROS_INFO_STREAM_GREEN("distanceInVector "<<distanceInVector);
    Point initPoint = getPointInDistance(basePoint, baseLineForSweep, distanceInVector);
    // ROS_INFO_STREAM_GREEN("initPoint "<<initPoint);
    Point robotStartPoint_first_last = getPointInDistFromVector(initPoint, width_on_ground_w_overlap / 2.0 + radius / 2.0 + skip_arena_sides_displacement,
                                                                sweepAlongLine.getDirection());  // baseLineForSweep.getPointInDistance(distanceInVector);

    Point robotStartPoint = getPointInDistFromVector(initPoint, width_on_ground_w_overlap / 2.0 + radius / 2.0 + skip_arena_sides_displacement,
                                                     sweepAlongLine.getDirection());  // baseLineForSweep.getPointInDistance(distanceInVector);

    // ROS_INFO_STREAM_GREEN("robotStartPoint "<<robotStartPoint);
    double initY = 0;
    // ROS_INFO_STREAM("initPointFOVOfsetted "<<initPointFOVOfsetted);
    for (int var = 0; var < num_sweeps_per_robot; ++var) {

      ROS_INFO_STREAM("sweep num " << var);
      // if ((var % 2 == 0 && robotID % 2 == 0) || (var % 1 == 0 && robotID % 2 == 0)) {
      if ((var % 2 == 0 && robotID % 2 == 0) || (var % 2 == 1 && robotID % 2 == 1)) {

        ROS_INFO_STREAM("if");
        double ang_n                 = sweepAlongLine.getDirection().getAngle();  // + flight_yaw;
        double ang                   = normalizeAngle(ang_n, 0, M_2PI);
        Point  singleSweepStartPoint = getPointInDistFromVector(robotStartPoint, width_sweep * var, baseLineForSweep.getDirection());

        // move it using radius_rate_shift_first_last
        if (!loop && var == 0) {
          singleSweepStartPoint = getPointInDistFromVector(robotStartPoint_first_last, width_sweep * var, baseLineForSweep.getDirection());
        }

        Point singleSweepEndPoint =
            getPointInDistFromVector(singleSweepStartPoint, sweepAlongLine.getLength() - width_on_ground_w_overlap - radius - 2 * skip_arena_sides_displacement,
                                     sweepAlongLine.getDirection());

        // move it using radius_rate_shift_first_last
        if (!loop && (var == 0 || num_sweeps_per_robot - 1)) {
          singleSweepEndPoint = getPointInDistFromVector(singleSweepStartPoint,
                                                         sweepAlongLine.getLength() - width_on_ground_w_overlap - radius - 2 * skip_arena_sides_displacement,
                                                         sweepAlongLine.getDirection());
        }

        // robotSweepPoints[robotID].push_back(State(singleSweepStartPoint, ang));
        // robotSweepPoints[robotID].push_back(State(singleSweepEndPoint, ang));
        ROS_INFO_STREAM("ang " << ang);
        arenaBorderScanPoints[robotID].push_back(GraphNode_AngNeigh(singleSweepStartPoint, ang, 0, radius));
        arenaBorderScanPoints[robotID].push_back(GraphNode_AngNeigh(singleSweepEndPoint, ang, 0, radius));

      } else {
        ROS_INFO_STREAM("else");
        double ang_n = sweepAlongLine.getDirection().getAngle() + M_PI;  // + flight_yaw;

        double ang                   = normalizeAngle(ang_n, 0, M_2PI);
        Point  singleSweepStartPoint = getPointInDistFromVector(robotStartPoint, width_sweep * var, baseLineForSweep.getDirection());

        // move it using radius_rate_shift_first_last
        if (!loop && var == num_sweeps_per_robot - 1) {
          singleSweepStartPoint = getPointInDistFromVector(robotStartPoint_first_last, width_sweep * var, baseLineForSweep.getDirection());
        }

        Point singleSweepEndPoint =
            getPointInDistFromVector(singleSweepStartPoint, sweepAlongLine.getLength() - width_on_ground_w_overlap - radius - 2 * skip_arena_sides_displacement,
                                     sweepAlongLine.getDirection());

        // move it using radius_rate_shift_first_last
        if (!loop && (var == 0 || var == num_sweeps_per_robot - 1)) {
          singleSweepEndPoint = getPointInDistFromVector(singleSweepStartPoint,
                                                         sweepAlongLine.getLength() - width_on_ground_w_overlap - radius - 2 * skip_arena_sides_displacement,
                                                         sweepAlongLine.getDirection());
        }

        // robotSweepPoints[robotID].push_back(State(singleSweepEndPoint, ang));
        // robotSweepPoints[robotID].push_back(State(singleSweepStartPoint, ang));
        ROS_INFO_STREAM("ang " << ang);
        arenaBorderScanPoints[robotID].push_back(GraphNode_AngNeigh(singleSweepEndPoint, ang, 0, radius));
        arenaBorderScanPoints[robotID].push_back(GraphNode_AngNeigh(singleSweepStartPoint, ang, 0, radius));
      }
    }
    if (loop) {
      // add the first waypoint to the end if loop
      // robotSweepPoints[robotID].push_back(robotSweepPoints[robotID][0]);
      arenaBorderScanPoints[robotID].push_back(arenaBorderScanPoints[robotID][0]);
    }
  }

  DubinsTrajectory dt_sampler(maxScanningSpeed, maxScanningAcc);
  for (int robotID = 0; robotID < arenaBorderScanPoints.size(); ++robotID) {
    for (int var = 0; var < arenaBorderScanPoints[robotID].size(); ++var) {
      ROS_INFO_STREAM("r" << robotID << " scan point " << arenaBorderScanPoints[robotID][var].node << " ang " << arenaBorderScanPoints[robotID][var].ang);
    }
    SamplesWithTime<opendubins::State> res = dt_sampler.sample_trajectory_dubins(arenaBorderScanPoints[robotID]);
    ROS_INFO_STREAM("returned samples " << res.samples.size());
    sampledPositions[robotID] = res.samples;
    ROS_INFO_STREAM("huaaa " << sampledPositions[robotID].size());
  }

  return sampledPositions;
}

void ArenaScanPlanner::savePathToFile(std::vector<State> &toSave, std::string filename) {
  std::ofstream out(filename);
  if (out.is_open()) {
    for (int var = 0; var < toSave.size(); ++var) {
      out << toSave[var].point.x << " " << toSave[var].point.y << " " << toSave[var].ang << std::endl;
    }
    out.close();
  } else {
    std::cerr << "Cannot open " << filename << std::endl;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sweeping_planner");
  ROS_INFO_STREAM("main loop of sweeping_planner");
  ros::NodeHandle private_node_handle_("~");

  std::vector<float> tempList;
  // safety area
  // load the main system matrix
  ROS_INFO_STREAM("loading configs");
  private_node_handle_.getParam("arena_corners", tempList);
  ROS_INFO_STREAM("arena_corners loaded");

  MatrixXd sweepAreaCorners = MatrixXd::Zero(4, 2);

  int tempIdx = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 2; j++) {
      sweepAreaCorners(i, j) = tempList[tempIdx++];
    }
  }
  ROS_INFO_STREAM("arena_corners are:");
  ROS_INFO_STREAM(sweepAreaCorners(0, 0) << " " << sweepAreaCorners(0, 1));
  ROS_INFO_STREAM(sweepAreaCorners(1, 0) << " " << sweepAreaCorners(1, 1));
  ROS_INFO_STREAM(sweepAreaCorners(2, 0) << " " << sweepAreaCorners(2, 1));
  ROS_INFO_STREAM(sweepAreaCorners(3, 0) << " " << sweepAreaCorners(3, 1));

  int    num_robots                    = 3;
  double fieldofview_deg               = 115;  // 115.0;
  double overlap                       = 0.2;
  double radius_rate_shift             = 0.5;
  double radius_rate_shift_first_last  = 0.2;
  double flight_yaw_deg                = 0;
  double mapping_ground_altitude       = 0;
  double skip_arena_sides_displacement = 4;
  ROS_INFO_STREAM("create sweepPlanner");

  private_node_handle_.getParam("fieldofview_deg", fieldofview_deg);
  ROS_INFO_STREAM("loaded fieldofview_deg " << fieldofview_deg);

  private_node_handle_.getParam("overlap", overlap);
  ROS_INFO_STREAM("loaded overlap " << overlap);

  private_node_handle_.getParam("radius_rate_shift", radius_rate_shift);
  ROS_INFO_STREAM("radius_rate_shift " << radius_rate_shift);

  private_node_handle_.getParam("radius_rate_shift_first_last", radius_rate_shift_first_last);
  ROS_INFO_STREAM("radius_rate_shift_first_last " << radius_rate_shift_first_last);

  private_node_handle_.getParam("flight_yaw_deg", flight_yaw_deg);
  ROS_INFO_STREAM("flight_yaw_deg " << flight_yaw_deg);
  double flight_yaw = (flight_yaw_deg / 180.0) * (M_2PI / 2.0);

  private_node_handle_.getParam("mapping_ground_altitude", mapping_ground_altitude);
  ROS_INFO_STREAM("mapping_ground_altitude " << mapping_ground_altitude);

  private_node_handle_.getParam("skip_arena_sides_displacement", skip_arena_sides_displacement);
  ROS_INFO_STREAM("skip_arena_sides_displacement " << skip_arena_sides_displacement);

  ArenaScanPlanner sweepPlanner(sweepAreaCorners, fieldofview_deg, overlap, radius_rate_shift, radius_rate_shift_first_last, flight_yaw,
                                mapping_ground_altitude, skip_arena_sides_displacement);

  ROS_INFO_STREAM("spinning");
  ros::spin();
  return 0;
}
