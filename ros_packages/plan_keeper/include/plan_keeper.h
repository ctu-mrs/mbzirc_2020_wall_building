#ifndef PLAN_KEEPER_H
#define PLAN_KEEPER_H

#include <ros/ros.h>
#include "plan_keeper/BrickPlan.h"
#include "plan_keeper/ChangeState.h"
#include "plan_keeper/BrickResult.h"
#include "plan_keeper/PlanDiagnostics.h"
#include "plan_keeper/PlanStateTypes.h"
#include "plan_keeper/WallState.h"
#include <mrs_msgs/Reference.h>
#include "plan_keeper/SetMappedPositions.h"
#include "brick_mapping/MappedArenaObjectsStamped.h"
#include "std_srvs/Trigger.h"

/*
#include <ctop_planner/RobotPlan.h>
#include "ctop_planner/WallDefinition.h"
#include "ctop_planner/BuildingRules.h"
*/

#include <math.h>
#include <mrs_lib/subscribe_handler.h>
#include <set>
#include "dataset_loader_wall_mbzirc.h"

typedef struct ConcurrentRule
{
  int16_t brick1;
  int16_t brick2;
} ConcurrentRule;


//== ( lhs.brick1 == rhs.brick1 && lhs.brick2 == rhs.brick2 ) || ( lhs.brick2 == rhs.brick1 && lhs.brick1 == rhs.brick2)
inline bool operator<(const ConcurrentRule& lhs, const ConcurrentRule& rhs) {
  int16_t min_id_l  = std::min(lhs.brick1, lhs.brick2);
  int16_t min_id_r  = std::min(rhs.brick1, rhs.brick2);
  int16_t max_id_l  = std::max(lhs.brick1, lhs.brick2);
  int16_t max_id_r  = std::max(rhs.brick1, rhs.brick2);
  bool    less_than = min_id_l < min_id_r || (min_id_l == min_id_r && max_id_l < max_id_r);
  // ROS_INFO_STREAM("less than is "<<less_than<<" for concurrence (" << lhs.brick1 << "<->" <<  lhs.brick2 << ") and (" <<  rhs.brick1 << "->" <<  rhs.brick2
  // << ")");
  return less_than;
}

typedef struct PrecedenceRule
{
  int16_t brick_before;
  int16_t brick_after;
} PrecedenceRule;

inline bool operator<(const PrecedenceRule& lhs, const PrecedenceRule& rhs) {
  bool less_than = lhs.brick_before < rhs.brick_before || (lhs.brick_before == rhs.brick_before && lhs.brick_after < rhs.brick_after);
  // ROS_INFO_STREAM("less than is "<<less_than<<" for precedence (" << lhs.brick_before << "->" << lhs.brick_after << ") and (" << rhs.brick_before << "->" <<
  // rhs.brick_after << ")");
  return less_than;
}

class PlanKeeper {
private:
  /* config parameters */
  std::string                                      _robot_name_;
  double                                           _object_drop_z_offset_;
  double                                           _object_drop_wait_z_offset_;
  double                                           _object_drop_yaw_offset_;
  std::vector<std::string>                         _robot_name_list_;
  std::vector<double>                              _robot_flight_altitude_list_;
  vector5                                          _red_brick_spot_;
  vector5                                          _green_brick_spot_;
  vector5                                          _blue_brick_spot_;
  vector5                                          _orange_brick_spot_;
  vector5                                          _wall_1_pos_;
  vector5                                          _wall_2_pos_;
  vector5                                          _wall_3_pos_;
  vector5                                          _wall_4_pos_;
  std::vector<vector5>                             _wall_poses_;
  std::vector<bool>                                _walls_mapped_;
  std::map<BrickType, bool>                        _brick_type_mapped_;
  int                                              _num_robots_;
  int                                              _robot_index_;
  double                                           _brick_pickup_spot_length_;
  double                                           _wall_yaw_randomize_along_;
  double                                           _wall_length_randomize_;
  double                                           _fly_altitude_;
  std::string                                      _diagnostics_topic_name_;
  std::string                                      _backup_folder_;
  WallDefinition                                   _wall_definition_;
  WallDefinition3D                                 _wall_definition_3D_;
  std::map<int, Brick3D>                           _brick_id_to_brick_;
  std::map<BrickType, int>                         _grasped_bricks_typed_count;
  std::map<BrickType, BrickSeeWallHavingBrickType> _see_wall_having_brick_map;
  std::map<BrickType, int>                         _num_bricks_one_line_typed;


  /* member parameters */
  std::vector<std::vector<Brick3D>>         robots_plans_;
  std::vector<int>                          robots_plans_indexes_;
  std::vector<std::string>                  state_names_;
  std::vector<plan_keeper::PlanDiagnostics> other_robot_diagnostics_;
  uint8_t                                   state_id_;
  std::vector<short int>                    placed_bricks_;

  /* wall definition and constraints params*/
  std::vector<mrs_msgs::Reference> brick_positions_;
  std::vector<short int>           brick_ids_;
  // std::vector<ctop_planner::BrickRules>     brick_rules_;
  std::vector<ConcurrentRule> all_concurrent_rules_;
  std::vector<PrecedenceRule> all_precedence_rules_;

  // publishers definition
  ros::Publisher pub_diagnostics_;
  ros::Publisher pub_wall_state_;
  ros::Publisher pub_mapped_arena_;

  // subscribers definition
  std::vector<mrs_lib::SubscribeHandlerPtr<plan_keeper::PlanDiagnostics>> sub_diagnostics_list_;

  // services definition
  ros::ServiceServer service_plan_;
  ros::ServiceServer service_get_brick_;
  ros::ServiceServer service_change_state_;
  ros::ServiceServer service_brick_placed_;
  ros::ServiceServer service_set_mapped_objects_;
  ros::ServiceServer service_wall_definition_;
  ros::ServiceServer service_building_rules_;
  ros::ServiceServer service_is_brick_placeable_;
  ros::ServiceServer service_load_mapped_arena_from_file_;

  // msg to be published once mapped
  brick_mapping::MappedArenaObjectsStamped mapped_arena_msg;
  int                                      num_map_received;
  bool                                     map_received;


  // timers definition
  ros::Timer diagnostics_timer_;

  // | -------------------subscriber callbacks ------------------- |
  void callbackDiagnostics(mrs_lib::SubscribeHandlerPtr<plan_keeper::PlanDiagnostics> sh_ptr);

  // | ------------------- service callbacks ------------------- |
  // bool callbackLoadPlan(plan_keeper::RobotPlan::Request& req, plan_keeper::RobotPlan::Response& res);
  bool callbackGetNextBrick(plan_keeper::BrickPlan::Request& req, plan_keeper::BrickPlan::Response& res);
  bool callbackChangeState(plan_keeper::ChangeState::Request& req, plan_keeper::ChangeState::Response& res);
  bool callbackBrickPlaced(plan_keeper::BrickResult::Request& req, plan_keeper::BrickResult::Response& res);
  bool callbackSetMappedObjects(plan_keeper::SetMappedPositions::Request& req, plan_keeper::SetMappedPositions::Response& res);
  bool callbackLoadMappedArenaFromFile(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  // bool callbackWallDefinition(plan_keeper::SetWallDefinition::Request& req, plan_keeper::SetWallDefinition::Response& res);
  // bool callbackBuildingRules(plan_keeper::SetBuildingRules::Request& req, plan_keeper::SetBuildingRules::Response& res);
  // bool callbackIsBrickPlaceable(plan_keeper::IsBrickPlaceable::Request& req, plan_keeper::IsBrickPlaceable::Response& res);

  // | ------------------- timers callbacks ------------------- |
  void diagnosticsTimer(const ros::TimerEvent& event);

  // | ------------------- member functions ------------------- |
  void publishDiagnostics();
  void publishWallState(const bool save_data);
  bool loadPlanBackup();
  void savePlanBackup();
  void calculateWallDefinition3DAndPlan();

  // bool checkBrickPlacable(int16_t current_brick_index);

public:
  PlanKeeper();
  ~PlanKeeper();
  void publish_arena_state();
};
#endif
