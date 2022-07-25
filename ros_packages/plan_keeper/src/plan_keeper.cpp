/* includes //{ */

#include "plan_keeper.h"
#include <algorithm>
#include <mrs_lib/param_loader.h>
#include <iostream>
#include <fstream>

//}

#define STRING_EQUAL 0
#define btoa(x) ((x) ? "true" : "false")
#define NAME_OF(v) #v

#define BACKUP_FILENAME "/data.txt"

using std::string;

/* PlanKeeper::PlanKeeper() //{ */

PlanKeeper::PlanKeeper() {
  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Time::waitForValid();
  num_map_received = 0;
  map_received     = false;
  ROS_INFO("[PlanKeeper]: Loading general parameters:");
  mrs_lib::ParamLoader param_loader(nh, "PlanKeeper");

  param_loader.loadParam("robot_name", _robot_name_, std::string());

  param_loader.loadParam("object_drop_z_offset", _object_drop_z_offset_);
  param_loader.loadParam("object_drop_yaw_offset", _object_drop_yaw_offset_);
  param_loader.loadParam("object_drop_wait_z_offset", _object_drop_wait_z_offset_);
  param_loader.loadParam("diagnostics_topic_name", _diagnostics_topic_name_);
  param_loader.loadParam("backup_folder", _backup_folder_);

  int diag_rate;
  param_loader.loadParam("diagnostics_rate", diag_rate);

  param_loader.loadParam("main/robot_name_list", _robot_name_list_);
  _num_robots_ = _robot_name_list_.size();

  param_loader.loadParam("main/brick_pickup_spot_length", _brick_pickup_spot_length_);

  param_loader.loadParam("main/wall_yaw_randomize_along", _wall_yaw_randomize_along_);
  param_loader.loadParam("main/wall_length_randomize", _wall_length_randomize_);

  param_loader.loadMatrixStatic<1, 5>("main/red_brick_spot", _red_brick_spot_);
  param_loader.loadMatrixStatic<1, 5>("main/green_brick_spot", _green_brick_spot_);
  param_loader.loadMatrixStatic<1, 5>("main/blue_brick_spot", _blue_brick_spot_);
  param_loader.loadMatrixStatic<1, 5>("main/orange_brick_spot", _orange_brick_spot_);

  param_loader.loadMatrixStatic<1, 5>("main/wall_1_pos", _wall_1_pos_);
  param_loader.loadMatrixStatic<1, 5>("main/wall_2_pos", _wall_2_pos_);
  param_loader.loadMatrixStatic<1, 5>("main/wall_3_pos", _wall_3_pos_);
  param_loader.loadMatrixStatic<1, 5>("main/wall_4_pos", _wall_4_pos_);

  param_loader.loadParam("main/flying_altitudes", _robot_flight_altitude_list_);

  _walls_mapped_.resize(4);
  _walls_mapped_[0]                      = true;
  _walls_mapped_[1]                      = true;
  _walls_mapped_[2]                      = true;
  _walls_mapped_[3]                      = true;
  _brick_type_mapped_[RED_BRICK_TYPE]    = true;
  _brick_type_mapped_[GREEN_BRICK_TYPE]  = true;
  _brick_type_mapped_[BLUE_BRICK_TYPE]   = true;
  _brick_type_mapped_[ORANGE_BRICK_TYPE] = true;

  param_loader.loadParam("main/num_red_bricks_one_line", _num_bricks_one_line_typed[RED_BRICK_TYPE]);
  param_loader.loadParam("main/num_green_bricks_one_line", _num_bricks_one_line_typed[GREEN_BRICK_TYPE]);
  param_loader.loadParam("main/num_blue_bricks_one_line", _num_bricks_one_line_typed[BLUE_BRICK_TYPE]);
  param_loader.loadParam("main/num_orange_bricks_one_line", _num_bricks_one_line_typed[ORANGE_BRICK_TYPE]);

  ROS_INFO_STREAM("[PlanKeeper]: _brick_pickup_spot_length_ " << _brick_pickup_spot_length_);

  ROS_INFO_STREAM("[PlanKeeper]: _num_bricks_one_line_typed[RED_BRICK_TYPE] " << _num_bricks_one_line_typed[RED_BRICK_TYPE]);
  ROS_INFO_STREAM("[PlanKeeper]: _num_bricks_one_line_typed[GREEN_BRICK_TYPE] " << _num_bricks_one_line_typed[GREEN_BRICK_TYPE]);
  ROS_INFO_STREAM("[PlanKeeper]: _num_bricks_one_line_typed[BLUE_BRICK_TYPE] " << _num_bricks_one_line_typed[BLUE_BRICK_TYPE]);
  ROS_INFO_STREAM("[PlanKeeper]: _num_bricks_one_line_typed[ORANGE_BRICK_TYPE] " << _num_bricks_one_line_typed[ORANGE_BRICK_TYPE]);

  _grasped_bricks_typed_count[RED_BRICK_TYPE]    = 0;
  _grasped_bricks_typed_count[GREEN_BRICK_TYPE]  = 0;
  _grasped_bricks_typed_count[BLUE_BRICK_TYPE]   = 0;
  _grasped_bricks_typed_count[ORANGE_BRICK_TYPE] = 0;

  // load mbzirc wall config
  std::string wall_config;
  param_loader.loadParam("wall_config_file", wall_config);
  _wall_definition_ = DatasetLoaderWallMBZIRC::loadDataset(wall_config);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[PlanKeeper]: Could not load all non-optional parameters!");
    ros::shutdown();
    return;
  }

  /* sanity checks //{ */

  if (_robot_name_list_.empty()) {
    ROS_ERROR("[PlanKeeper]: robot_name_list (target robots) is empty!");
    ros::shutdown();
    return;
  }

  _robot_index_  = -1;
  _fly_altitude_ = 4;
  for (int var = 0; var < _num_robots_; ++var) {
    if (_robot_name_list_[var].compare(_robot_name_) == STRING_EQUAL) {
      _robot_index_ = var;
      ROS_INFO_STREAM("[PlanKeeper]: This robot index is " << _robot_index_);
      break;
    }
  }
  if (_robot_index_ == -1) {
    ROS_ERROR("[PlanKeeper]: robot_name_list doesn't contain this ROBOT_NAME!");
    ros::shutdown();
    return;
  }

  if (_robot_index_ < _robot_flight_altitude_list_.size()) {
    _fly_altitude_ = _robot_flight_altitude_list_[_robot_index_];
  } else {
    ROS_ERROR("[PlanKeeper]: _fly_altitude_ for this robot is not defined");
    ros::shutdown();
    return;
  }

  if (_num_robots_ == -1) {
    ROS_ERROR("[PlanKeeper]: robot_name_list (target robots) is empty!");
    ros::shutdown();
    return;
  }

  //}

  _see_wall_having_brick_map[RED_BRICK_TYPE]   = BRICK_SEE_WALL_HAVING_RED;
  _see_wall_having_brick_map[GREEN_BRICK_TYPE] = BRICK_SEE_WALL_HAVING_GREEN;
  _see_wall_having_brick_map[BLUE_BRICK_TYPE]  = BRICK_SEE_WALL_HAVING_BLUE;

  if (param_loader.loadedSuccessfully()) {
    ROS_INFO("[PlanKeeper]: All parameters loaded.");
    ROS_INFO("[PlanKeeper]: ------------------------------------------------");
  } else {
    ROS_INFO("[PlanKeeper]: Some parameters loaded badly.");
    ROS_INFO("[PlanKeeper]: ------------------------------------------------");
  }

  this->calculateWallDefinition3DAndPlan();

  std::vector<Brick3D> this_robot_plan = robots_plans_[_robot_index_];
  ROS_INFO("[PlanKeeper]: this robot plan is:");

  for (int var = 0; var < this_robot_plan.size(); ++var) {
    Brick3D &brick = this_robot_plan[var];
    ROS_INFO_STREAM("            : " << DatasetLoaderWallMBZIRC::brick_names[brick.type] << ",id:" << brick.id << ",pos:(" << brick.x << "," << brick.y << ")");
  }

  robots_plans_indexes_.resize(_num_robots_);
  for (int var = 0; var < _num_robots_; ++var) {
    robots_plans_indexes_[var] = -1;
  }

  // prepare the array of names
  // IMPORTANT, update this with each update of the PlanStateType message
  state_names_.push_back(NAME_OF(plan_keeper::PlanStateTypes::IDLE));
  state_names_.push_back(NAME_OF(plan_keeper::PlanStateTypes::PREPARIMG));
  state_names_.push_back(NAME_OF(plan_keeper::PlanStateTypes::FLYING));
  state_names_.push_back(NAME_OF(plan_keeper::PlanStateTypes::GRASPING));
  state_names_.push_back(NAME_OF(plan_keeper::PlanStateTypes::PLACING));
  state_names_.push_back(NAME_OF(plan_keeper::PlanStateTypes::WAITING));

  ROS_WARN("[PlanKeeper]: SAFETY Checking the State2Name conversion. If it fails here, you should update the code above this ROS_INFO");
  for (int i = 0; i < plan_keeper::PlanStateTypes::TYPE_COUNT; i++) {
    std::size_t found = state_names_[i].find_last_of(":");
    state_names_[i]   = state_names_[i].substr(found + 1);
    ROS_INFO("[PlanKeeper]: state_type[%d]=%s", i, state_names_[i].c_str());
  }
  ROS_INFO("[PlanKeeper]: ------------------------------------------------");

  state_id_ = plan_keeper::PlanStateTypes::IDLE;

  // create publishers
  pub_diagnostics_  = nh.advertise<plan_keeper::PlanDiagnostics>("diagnostics_out", 1);
  pub_mapped_arena_ = nh.advertise<brick_mapping::MappedArenaObjectsStamped>("mapped_arena_out", 1);
  pub_wall_state_   = nh.advertise<plan_keeper::WallState>("wall_state_out", 1);

  // create subscribers
  /* mrs_lib::SubscribeMgr smgr(nh, "PlanKeeper"); */

  /*
   string topic;
   for (int i = 0; i < _num_robots_; ++i) {
   if (i != _robot_index_) {
   topic = "/" + _robot_name_list_[i] + "/" + _diagnostics_topic_name_;
   auto sub = smgr.create_handler<plan_keeper::PlanDiagnostics>(topic, ros::Duration(3 * diag_rate), { }, this,
   &PlanKeeper::callbackDiagnostics, this,
   false);//false==not thread safe

   sub_diagnostics_list_.push_back(sub);
   }

   plan_keeper::PlanDiagnostics empty_diag;
   empty_diag.current_brick_id = -1;
   other_robot_diagnostics_.push_back(empty_diag);
   }
   */

  // create service servers
  // service_plan_ = nh.advertiseService("set_complete_plan_in", &PlanKeeper::callbackLoadPlan, this);
  service_get_brick_                   = nh.advertiseService("get_brick_plan_in", &PlanKeeper::callbackGetNextBrick, this);
  service_change_state_                = nh.advertiseService("change_state_in", &PlanKeeper::callbackChangeState, this);
  service_brick_placed_                = nh.advertiseService("brick_placed_in", &PlanKeeper::callbackBrickPlaced, this);
  service_set_mapped_objects_          = nh.advertiseService("set_mapped_objects_in", &PlanKeeper::callbackSetMappedObjects, this);
  service_load_mapped_arena_from_file_ = nh.advertiseService("load_mapped_arena_from_file", &PlanKeeper::callbackLoadMappedArenaFromFile, this);
  // service_wall_definition_ = nh.advertiseService("wall_definition_in", &PlanKeeper::callbackWallDefinition, this);
  // service_building_rules_ = nh.advertiseService("building_rules_in", &PlanKeeper::callbackBuildingRules, this);
  // service_is_brick_placeable_ = nh.advertiseService("is_brick_placeable", &PlanKeeper::callbackIsBrickPlaceable, this);

  // create timers
  diagnostics_timer_ = nh.createTimer(ros::Rate(diag_rate), &PlanKeeper::diagnosticsTimer, this);

  ROS_INFO("[PlanKeeper]: end init ------------------------------------------------");
}

//}

/* PlanKeeper::~PlanKeeper() //{ */

PlanKeeper::~PlanKeeper() {
}

//}

/* //{ PlanKeeper::callbackDiagnostics(...) */

/* void PlanKeeper::callbackDiagnostics(const plan_keeper::PlanDiagnostics &msg) { */
void PlanKeeper::callbackDiagnostics(mrs_lib::SubscribeHandler<plan_keeper::PlanDiagnostics> sh_ptr) {
  /* ROS_INFO_STREAM("[PlanKeeper]: callbackDiagnostics begin"); */

  auto msg = sh_ptr.getMsg();

  std::vector<string>::iterator it;

  it = std::find(_robot_name_list_.begin(), _robot_name_list_.end(), msg->robot_name);
  if (it != _robot_name_list_.end()) {
    int  index            = it - _robot_name_list_.begin();
    bool new_brick_placed = other_robot_diagnostics_[index].placed_brick_ids.size() != msg->placed_brick_ids.size();

    if (other_robot_diagnostics_[index].current_brick_id != msg->current_brick_id || new_brick_placed) {
      // print debug when current brick changed or number placed changed
      ROS_INFO_STREAM("[PlanKeeper]: " << msg->robot_name << ": Received diagnostics with new data");
      ROS_INFO_STREAM("[PlanKeeper]: " << msg->robot_name << ": current brick_id: " << msg->current_brick_id);

      std::stringstream ss;
      for (int16_t brick_id : msg->placed_brick_ids) {
        ss << " " << brick_id;
      }
      ROS_INFO_STREAM("[PlanKeeper]: " << msg->robot_name << ": placed bricks: [" << ss.str() << "]");
    }

    other_robot_diagnostics_[index] = *msg;

    if (new_brick_placed) {
      publishWallState(true);
    }

  } else {
    ROS_ERROR("[PlanKeeper]: Received diagnostics from unknown robot. %s", msg->robot_name.c_str());
  }
  /* ROS_INFO_STREAM("[PlanKeeper]: callbackDiagnostics end"); */
}

//}

/* PlanKeeper::callbackBrickPlaced() //{ */

bool PlanKeeper::callbackBrickPlaced(plan_keeper::BrickResult::Request &req, plan_keeper::BrickResult::Response &res) {
  ROS_INFO_STREAM("[PlanKeeper]: callbackBrickPlaced");
  char tempStr[100];
  if (req.result) {

    sprintf((char *)&tempStr, "Brick %d placing has been saved.", req.brick_id);
    placed_bricks_.push_back(req.brick_id);
    Brick3D   brick = _brick_id_to_brick_[req.brick_id];
    BrickType type  = brick.type;
    _grasped_bricks_typed_count[type] += 1;
    ROS_INFO_STREAM("[PlanKeeper]: callbackBrickPlaced placed brick id " << req.brick_id);
    publishDiagnostics();
    publishWallState(true);
    savePlanBackup();
  } else {
    ROS_INFO_STREAM("[PlanKeeper]: brick id " << req.brick_id << " not placed !!!!!");
    sprintf((char *)&tempStr, "Brick %d placing has not been sucessful.", req.brick_id);
  }

  res.success = true;
  res.message = tempStr;

  ROS_INFO("[PlanKeeper]: %s", res.message.c_str());

  return true;
}

//}

bool PlanKeeper::callbackLoadMappedArenaFromFile(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  bool results = this->loadPlanBackup();
  res.success  = results;
  std::stringstream ss;
  ss << "called load mapped area with result " << res.success;
  res.message = ss.str();
  return true;
}

/* PlanKeeper::callbackSetMappedObjects() //{ */

bool PlanKeeper::callbackSetMappedObjects(plan_keeper::SetMappedPositions::Request &req, plan_keeper::SetMappedPositions::Response &res) {
  ROS_INFO_STREAM("received mapped objects");
  // req.mapped_objects
  /*brick_mapping/MappedArenaObjects mapped_objects
   mbzirc_msgs/ObjectWithType wall1
   mbzirc_msgs/ObjectWithType wall2
   mbzirc_msgs/ObjectWithType wall3
   mbzirc_msgs/ObjectWithType wall4
   mbzirc_msgs/ObjectWithType bricks_red
   mbzirc_msgs/ObjectWithType bricks_green
   mbzirc_msgs/ObjectWithType bricks_blue
   */

  ROS_INFO_STREAM("req.mapped_objects");
  ROS_INFO_STREAM(req.mapped_objects);

  mapped_arena_msg.header.frame_id = "gps_origin";
  mapped_arena_msg.header.seq      = num_map_received;
  mapped_arena_msg.header.stamp    = ros::Time::now();
  num_map_received++;
  mapped_arena_msg.mapped_objects = req.mapped_objects;
  map_received                    = true;
  ROS_INFO_STREAM("should publish received map now");

  if (req.mapped_objects.wall1.type != 0) {
    _wall_1_pos_(0)   = req.mapped_objects.wall1.x;
    _wall_1_pos_(1)   = req.mapped_objects.wall1.y;
    _wall_1_pos_(2)   = req.mapped_objects.wall1.z;
    _wall_1_pos_(3)   = req.mapped_objects.wall1.yaw;
    _wall_1_pos_(4)   = req.mapped_objects.wall1.len;
    _walls_mapped_[0] = true;
  } else {
    _walls_mapped_[0] = false;
  }

  if (req.mapped_objects.wall2.type != 0) {
    _wall_2_pos_(0)   = req.mapped_objects.wall2.x;
    _wall_2_pos_(1)   = req.mapped_objects.wall2.y;
    _wall_2_pos_(2)   = req.mapped_objects.wall2.z;
    _wall_2_pos_(3)   = req.mapped_objects.wall2.yaw;
    _wall_2_pos_(4)   = req.mapped_objects.wall2.len;
    _walls_mapped_[1] = true;
  } else {
    _walls_mapped_[1] = false;
  }

  if (req.mapped_objects.wall3.type != 0) {
    _wall_3_pos_(0)   = req.mapped_objects.wall3.x;
    _wall_3_pos_(1)   = req.mapped_objects.wall3.y;
    _wall_3_pos_(2)   = req.mapped_objects.wall3.z;
    _wall_3_pos_(3)   = req.mapped_objects.wall3.yaw;
    _wall_3_pos_(4)   = req.mapped_objects.wall3.len;
    _walls_mapped_[2] = true;
  } else {
    _walls_mapped_[2] = false;
  }

  if (req.mapped_objects.wall4.type != 0) {
    _wall_4_pos_(0)   = req.mapped_objects.wall4.x;
    _wall_4_pos_(1)   = req.mapped_objects.wall4.y;
    _wall_4_pos_(3)   = req.mapped_objects.wall4.z;
    _wall_4_pos_(3)   = req.mapped_objects.wall4.yaw;
    _wall_4_pos_(4)   = req.mapped_objects.wall4.len;
    _walls_mapped_[3] = true;
  } else {
    _walls_mapped_[3] = false;
  }

  if (req.mapped_objects.bricks_red.type != 0) {
    _red_brick_spot_(0)                 = req.mapped_objects.bricks_red.x;
    _red_brick_spot_(1)                 = req.mapped_objects.bricks_red.y;
    _red_brick_spot_(2)                 = 3;
    _red_brick_spot_(3)                 = req.mapped_objects.bricks_red.yaw;
    _red_brick_spot_(4)                 = req.mapped_objects.bricks_red.len;
    _brick_type_mapped_[RED_BRICK_TYPE] = true;
  } else {
    _brick_type_mapped_[GREEN_BRICK_TYPE] = false;
  }

  if (req.mapped_objects.bricks_green.type != 0) {
    _green_brick_spot_(0)                 = req.mapped_objects.bricks_green.x;
    _green_brick_spot_(1)                 = req.mapped_objects.bricks_green.y;
    _green_brick_spot_(2)                 = 3;
    _green_brick_spot_(3)                 = req.mapped_objects.bricks_green.yaw;
    _green_brick_spot_(4)                 = req.mapped_objects.bricks_green.len;
    _brick_type_mapped_[GREEN_BRICK_TYPE] = true;
  } else {
    _brick_type_mapped_[GREEN_BRICK_TYPE] = false;
  }

  if (req.mapped_objects.bricks_blue.type != 0) {
    _blue_brick_spot_(0)                 = req.mapped_objects.bricks_blue.x;
    _blue_brick_spot_(1)                 = req.mapped_objects.bricks_blue.y;
    _blue_brick_spot_(2)                 = 3;
    _blue_brick_spot_(3)                 = req.mapped_objects.bricks_blue.yaw;
    _blue_brick_spot_(4)                 = req.mapped_objects.bricks_blue.len;
    _brick_type_mapped_[BLUE_BRICK_TYPE] = true;
  } else {
    _brick_type_mapped_[BLUE_BRICK_TYPE] = false;
  }

  ROS_INFO_STREAM("[PlanKeeper]: _red_brick_spot_" << _red_brick_spot_);
  ROS_INFO_STREAM("[PlanKeeper]: _green_brick_spot_" << _green_brick_spot_);
  ROS_INFO_STREAM("[PlanKeeper]: _blue_brick_spot_" << _blue_brick_spot_);
  ROS_INFO_STREAM("[PlanKeeper]: _wall_1_pos_" << _wall_1_pos_);
  ROS_INFO_STREAM("[PlanKeeper]: _wall_2_pos_" << _wall_2_pos_);
  ROS_INFO_STREAM("[PlanKeeper]: _wall_3_pos_" << _wall_3_pos_);
  ROS_INFO_STREAM("[PlanKeeper]: _wall_4_pos_" << _wall_4_pos_);

  this->calculateWallDefinition3DAndPlan();

  res.success = true;
  res.message = "mapped objects set to the plan keeper";
  savePlanBackup();
  ROS_INFO("[PlanKeeper]: %s", res.message.c_str());
  return true;
}

//}

/* PlanKeeper::publish_arena_state() //{ */

void PlanKeeper::publish_arena_state() {
  if (map_received) {
    ROS_INFO_STREAM_THROTTLE(4, "[PlanKeeper]: publish arena state");
    pub_mapped_arena_.publish(mapped_arena_msg);
  }
}

//}

/* PlanKeeper::calculateWallDefinition3DAndPlan() //{ */

void PlanKeeper::calculateWallDefinition3DAndPlan() {
  ROS_WARN_STREAM("creating wall brick poses from wall poisionts");
  _wall_poses_.clear();
  robots_plans_.clear();
  _wall_poses_.push_back(_wall_1_pos_);
  _wall_poses_.push_back(_wall_2_pos_);
  _wall_poses_.push_back(_wall_3_pos_);
  _wall_poses_.push_back(_wall_4_pos_);

  _wall_definition_3D_       = DatasetLoaderWallMBZIRC::wallDefinitionTo3DPopsitions(_wall_definition_, _wall_poses_);
  std::string string_wall_3d = DatasetLoaderWallMBZIRC::getDefinition3DString(_wall_definition_3D_);
  ROS_INFO_STREAM("wall definition 3d:");
  ROS_INFO_STREAM(string_wall_3d);

  // fill the brick plan
  WallDefinition3D::iterator it_ch;
  BrickChannel3D::iterator   it_lay;
  for (it_ch = _wall_definition_3D_.begin(); it_ch != _wall_definition_3D_.end(); it_ch++) {
    int channel = it_ch->first;
    robots_plans_.push_back(std::vector<Brick3D>());
    for (it_lay = it_ch->second.begin(); it_lay != it_ch->second.end(); it_lay++) {
      int          layer  = it_lay->first;
      BrickLayer3D bricks = it_lay->second;
      for (int var = 0; var < bricks.size(); ++var) {
        robots_plans_[robots_plans_.size() - 1].push_back(bricks[var]);
        _brick_id_to_brick_[bricks[var].id] = bricks[var];
      }
    }
  }
}

//}

bool PlanKeeper::callbackGetNextBrick([[maybe_unused]] plan_keeper::BrickPlan::Request &req, plan_keeper::BrickPlan::Response &res) {

  std::vector<Brick3D> plan = robots_plans_[_robot_index_];
  ROS_INFO_STREAM("callbackGetNextBrick begin");
  ROS_INFO_STREAM("plan size " << plan.size());

  if (robots_plans_indexes_[_robot_index_] + 1 >= int(plan.size())) {
    res.success  = false;
    res.finished = true;
    res.message  = "Plan has been already finished!";
    ROS_ERROR("[PlanKeeper]: THIS_UAV: %s", res.message.c_str());
    return true;
  }

  robots_plans_indexes_[_robot_index_] = robots_plans_indexes_[_robot_index_] + 1;

  // skip blue brick is while not red and not green then index+=1
  while (robots_plans_indexes_[_robot_index_] < int(plan.size()) && plan[robots_plans_indexes_[_robot_index_]].type != RED_BRICK_TYPE &&
         plan[robots_plans_indexes_[_robot_index_]].type != GREEN_BRICK_TYPE) {
    ROS_WARN_STREAM("switching to next brick in case of non-red and non-green brick");
    robots_plans_indexes_[_robot_index_] = robots_plans_indexes_[_robot_index_] + 1;
  }

  if (robots_plans_indexes_[_robot_index_] >= int(plan.size())) {
    res.success  = false;
    res.finished = true;
    res.message  = "Plan has been already finished!";
    ROS_ERROR("[PlanKeeper]: THIS_UAV: %s", res.message.c_str());
    return true;
  }

  int &brick_index = robots_plans_indexes_[_robot_index_];
  ROS_INFO_STREAM("current brick_index is " << brick_index);
  Brick3D &brick = plan[brick_index];
  ROS_INFO_STREAM("callbackGetNextBrick next brick is of type " << brick.type << " with id " << brick.id);


  double move_drop_rand = (_wall_length_randomize_ * rand() / RAND_MAX) - (_wall_length_randomize_ / 2.0);
  ROS_INFO_STREAM("callbackGetNextBrick _wall_yaw_randomize_along_ " << _wall_yaw_randomize_along_);
  ROS_INFO_STREAM("callbackGetNextBrick _wall_length_randomize_ " << _wall_length_randomize_);
  ROS_INFO_STREAM("callbackGetNextBrick move_drop_rand " << move_drop_rand);

  res.brick_id = brick.id;
  mrs_msgs::Reference drop_position;
  drop_position.position.x = brick.wall_x + 0.5 * cos(brick.wall_yaw - M_PI_2) + move_drop_rand * cos(_wall_yaw_randomize_along_);
  drop_position.position.y = brick.wall_y + 0.5 * sin(brick.wall_yaw - M_PI_2) + move_drop_rand * sin(_wall_yaw_randomize_along_);
  drop_position.position.z = brick.wall_z;
  drop_position.heading    = brick.wall_yaw + _object_drop_yaw_offset_;

  res.brick_type           = brick.type;
  res.brick_see_wall_typed = _see_wall_having_brick_map[brick.type];
  res.wall_layer           = brick.layer - 1;  // peter wants 0-1 , I have 1-2

  res.next_to_wall_wait_position            = drop_position;
  res.next_to_wall_wait_position.position.z = _fly_altitude_;

  if (_walls_mapped_[0] && _walls_mapped_[1]) {
    double wall_centers_yaw     = atan2(_wall_1_pos_(1) - _wall_2_pos_(1), _wall_1_pos_(0) - _wall_2_pos_(0));
    double yaw_wait_before_wall = wall_centers_yaw;
    if (_robot_index_ % 2 == 0) {
      yaw_wait_before_wall += M_PI_2;
    } else {
      yaw_wait_before_wall -= M_PI_2;
    }
    res.next_to_wall_wait_position.position.x += 3.0 * cos(yaw_wait_before_wall);
    res.next_to_wall_wait_position.position.y += 3.0 * sin(yaw_wait_before_wall);
  }

  res.drop_wait_position            = drop_position;
  res.drop_wait_position.position.z = _fly_altitude_;
  ;

  // offset in yaw and z
  res.drop_position = drop_position;
  res.drop_position.position.z += _object_drop_z_offset_;

  double ang_displacement = 0;
  switch (res.brick_type) {
    case BrickType::RED_BRICK_TYPE:

      res.grasp_position.position.x = _red_brick_spot_(0);
      res.grasp_position.position.y = _red_brick_spot_(1);
      res.grasp_position.position.z = _red_brick_spot_(2);
      ang_displacement              = _red_brick_spot_(3);

      break;
    case BrickType::GREEN_BRICK_TYPE:

      res.grasp_position.position.x = _green_brick_spot_(0);
      res.grasp_position.position.y = _green_brick_spot_(1);
      res.grasp_position.position.z = _green_brick_spot_(2);
      ang_displacement              = _green_brick_spot_(3);

      break;
    case BrickType::BLUE_BRICK_TYPE:

      res.grasp_position.position.x = _blue_brick_spot_(0);
      res.grasp_position.position.y = _blue_brick_spot_(1);
      res.grasp_position.position.z = _blue_brick_spot_(2);
      ang_displacement              = _blue_brick_spot_(3);

      break;
    case BrickType::ORANGE_BRICK_TYPE:

      res.grasp_position.position.x = _orange_brick_spot_(0);
      res.grasp_position.position.y = _orange_brick_spot_(1);
      res.grasp_position.position.z = _orange_brick_spot_(2);
      ang_displacement              = _orange_brick_spot_(3);
      break;
    default:

      ROS_WARN("[PlanKeeper]: THIS_UAV:  UNKNOWN TYPE OF BRICK: %d", res.brick_type);
      res.success = false;
      res.message = "UNKNOWN TYPE OF BRICK!";

      return true;
      break;
  }

  ROS_INFO_STREAM("[PlanKeeper]: middle of position without displacement is " << res.grasp_position.position.x << " " << res.grasp_position.position.y << " "
                                                                              << res.grasp_position.position.z);

  // based on the _robot_index_ add some displacement to grasp position along brick grasp line
  ROS_INFO_STREAM("[PlanKeeper]: _grasped_bricks_typed_count[brick.type] " << _grasped_bricks_typed_count[brick.type]);
  ROS_INFO_STREAM("[PlanKeeper]: _num_bricks_one_line_typed[brick.type] " << _num_bricks_one_line_typed[brick.type]);

  double displacement_from_middle = 0;
  // TODO fix this!!!!!
  if (_num_robots_ > 1) {
    if (_num_robots_ == 2) {
      displacement_from_middle = _brick_pickup_spot_length_ / ((double)(_num_robots_));
    }

    if (_num_robots_ == 3) {
      displacement_from_middle = _brick_pickup_spot_length_ / ((double)(_num_robots_ - 1));
    }

    double len_per_brick = _brick_pickup_spot_length_ / ((double)_num_bricks_one_line_typed[brick.type]);
    displacement_from_middle -= len_per_brick * _grasped_bricks_typed_count[brick.type];
    ROS_INFO_STREAM("[PlanKeeper]: len_per_brick is " << len_per_brick);
  }
  // compensate number of grasped types of bricks

  ROS_INFO_STREAM("[PlanKeeper]: !!!!!!!!!  GOING FOR BRICK TYPE " << brick.type << "  !!!!!!!!!");
  ROS_INFO_STREAM("[PlanKeeper]: !!!!!!!!!  GOING FOR BRICK ID " << brick.id << "  !!!!!!!!!");
  ROS_INFO_STREAM("[PlanKeeper]: _brick_pickup_spot_length_ " << _brick_pickup_spot_length_);
  ROS_INFO_STREAM("[PlanKeeper]: displacement_from_middle is " << displacement_from_middle);

  // compensate uav index in the grasping position
  if (_num_robots_ > 1) {

    if (_num_robots_ == 2) {
      // start from both sides - results +-displacement_from_middle
      // (_robot_index_ - 0.5)*2
      double final_displace = 2.0 * (((double)_robot_index_) - 0.5);
      res.grasp_position.position.x += final_displace * displacement_from_middle * cos(ang_displacement);
      res.grasp_position.position.y += final_displace * displacement_from_middle * sin(ang_displacement);
    }
    if (_num_robots_ == 3) {
      // start from both sides - results +displ 0*displ -displ
      // (_robot_index_ - 1.0)*
      double final_displace = ((double)_robot_index_ - 1.0) * displacement_from_middle;
      ROS_INFO_STREAM("[PlanKeeper]: final_displace " << final_displace);
      res.grasp_position.position.x += final_displace * cos(ang_displacement);
      res.grasp_position.position.y += final_displace * sin(ang_displacement);
    }
  }
  ROS_INFO_STREAM("[PlanKeeper]: grasp position with displacement is " << res.grasp_position.position.x << " " << res.grasp_position.position.y << " "
                                                                       << res.grasp_position.position.z << " " << res.grasp_position.heading);
  ROS_INFO_STREAM("[PlanKeeper]: drop wait position with displacement is " << res.drop_wait_position.position.x << " " << res.drop_wait_position.position.y
                                                                           << " " << res.drop_wait_position.position.z << " "
                                                                           << res.drop_wait_position.heading);
  ROS_INFO_STREAM("[PlanKeeper]: drop position with displacement is " << res.drop_position.position.x << " " << res.drop_position.position.y << " "
                                                                      << res.drop_position.position.z << " " << res.drop_position.heading);

  if (!_walls_mapped_[_robot_index_]) {
    res.success  = false;
    res.finished = false;
    std::stringstream ss;
    ss << "[PlanKeeper]: error wall idx " << _robot_index_ << " is not mapped";
    res.message = ss.str();
    ROS_ERROR_STREAM(ss.str());
    return true;
  }

  if (!_brick_type_mapped_[brick.type]) {
    res.success  = false;
    res.finished = false;
    std::stringstream ss;
    ss << "[PlanKeeper]: error brick type " << brick.type << " is not mapped";
    res.message = ss.str();
    ROS_ERROR_STREAM(ss.str());
    return true;
  }

  res.success  = true;
  res.finished = false;
  res.message  = "Everything OK.";
  ROS_INFO("[PlanKeeper]: THIS_UAV: Going for brick_id: %d  (brick_type: %d)", res.brick_id, res.brick_type);
  return true;
}

//}

/* //{ PlanKeeper::diagnosticsTimer() */

void PlanKeeper::diagnosticsTimer([[maybe_unused]] const ros::TimerEvent &event) {

  publishDiagnostics();

  publishWallState(false);
}

//}

/* //{ PlanKeeper::publishDiagnostics() */

void PlanKeeper::publishDiagnostics() {

  plan_keeper::PlanDiagnostics diag_msg;

  diag_msg.stamp            = ros::Time::now();
  diag_msg.robot_name       = _robot_name_;
  diag_msg.current_brick_id = -1;
  diag_msg.state.state_id   = state_id_;
  diag_msg.placed_brick_ids = placed_bricks_;

  // TODO: fill remaining time
  diag_msg.remaining_time_to_finish_task = 0;

  if (!robots_plans_.empty()) {
    int current_index = robots_plans_indexes_[_robot_index_];
    current_index--;  // current index is switch right after getting calling GetNextBrick so the current is the previous one
    std::vector<Brick3D> &plan = robots_plans_[_robot_index_];

    if (current_index > -1 && current_index < int(plan.size())) {
      diag_msg.current_brick_id = plan[current_index].id;
    }
  }

  try {
    pub_diagnostics_.publish(diag_msg);
  }
  catch (...) {
    ROS_ERROR("[PlanKeeper]: Exception caught during publishing topic %s.", pub_diagnostics_.getTopic().c_str());
  }
}
//}

/* //{ PlanKeeper::publishWallState() */

void PlanKeeper::publishWallState(const bool save_data) {

  plan_keeper::WallState state_msg;

  state_msg.stamp = ros::Time::now();

  state_msg.placed_brick_ids.insert(state_msg.placed_brick_ids.end(), placed_bricks_.begin(), placed_bricks_.end());

  for (int var = 0; var < int(other_robot_diagnostics_.size()); ++var) {
    state_msg.placed_brick_ids.insert(state_msg.placed_brick_ids.end(), other_robot_diagnostics_[var].placed_brick_ids.begin(),
                                      other_robot_diagnostics_[var].placed_brick_ids.end());
  }

  try {
    pub_wall_state_.publish(state_msg);
  }
  catch (...) {
    ROS_ERROR("[PlanKeeper]: Exception caught during publishing topic %s.", pub_wall_state_.getTopic().c_str());
  }
}
//}

/* //{ PlanKeeper::savePlanBackup(...) */

void PlanKeeper::savePlanBackup() {
  const std::string filename = _backup_folder_ + BACKUP_FILENAME;
  ROS_INFO_STREAM("[PlanKeeper]: try to savePlanBackup");
  std::ofstream myfile(filename.c_str());
  if (myfile.is_open()) {
    ROS_INFO_STREAM("[PlanKeeper]: save current robots_plans_indexes_[_robot_index_] " << robots_plans_indexes_[_robot_index_]);
    myfile << robots_plans_indexes_[_robot_index_] << std::endl;

    for (int var = 0; var < 5; ++var) {
      if (var != 0) {
        myfile << " ";
      }
      myfile << _wall_1_pos_(var);
    }
    myfile << std::endl;
    for (int var = 0; var < 5; ++var) {
      if (var != 0) {
        myfile << " ";
      }
      myfile << _wall_2_pos_(var);
    }
    myfile << std::endl;
    for (int var = 0; var < 5; ++var) {
      if (var != 0) {
        myfile << " ";
      }
      myfile << _wall_3_pos_(var);
    }
    myfile << std::endl;
    for (int var = 0; var < 5; ++var) {
      if (var != 0) {
        myfile << " ";
      }
      myfile << _wall_4_pos_(var);
    }
    myfile << std::endl;
    for (int var = 0; var < 5; ++var) {
      if (var != 0) {
        myfile << " ";
      }
      myfile << _red_brick_spot_(var);
    }
    myfile << std::endl;
    for (int var = 0; var < 5; ++var) {
      if (var != 0) {
        myfile << " ";
      }
      myfile << _green_brick_spot_(var);
    }
    myfile << std::endl;
    for (int var = 0; var < 5; ++var) {
      if (var != 0) {
        myfile << " ";
      }
      myfile << _blue_brick_spot_(var);
    }

    myfile.close();
  } else {
    ROS_ERROR("[PlanKeeper]: Unable to open file: \"%s\"", filename.c_str());
  };
}
//}

/* //{ PlanKeeper::loadPlanBackup() */

bool PlanKeeper::loadPlanBackup() {
  const std::string filename = _backup_folder_ + BACKUP_FILENAME;
  ROS_INFO("[PlanKeeper]: loading plan backup now");

  std::ifstream myfile(filename.c_str());
  if (myfile.is_open()) {

    placed_bricks_.clear();
    std::string line;
    std::getline(myfile, line);
    robots_plans_indexes_[_robot_index_] = stoi(line);

    std::getline(myfile, line);
    std::stringstream ss_w1(line);
    for (int var = 0; var < 5; ++var) {
      ss_w1 >> _wall_1_pos_(var);
    }

    std::getline(myfile, line);
    std::stringstream ss_w2(line);
    for (int var = 0; var < 5; ++var) {
      ss_w2 >> _wall_2_pos_(var);
    }

    std::getline(myfile, line);
    std::stringstream ss_w3(line);
    for (int var = 0; var < 5; ++var) {
      ss_w3 >> _wall_3_pos_(var);
    }

    std::getline(myfile, line);
    std::stringstream ss_w4(line);
    for (int var = 0; var < 5; ++var) {
      ss_w4 >> _wall_4_pos_(var);
    }

    std::getline(myfile, line);
    std::stringstream ss_red(line);
    for (int var = 0; var < 5; ++var) {
      ss_red >> _red_brick_spot_(var);
    }

    std::getline(myfile, line);
    std::stringstream ss_green(line);
    for (int var = 0; var < 5; ++var) {
      ss_green >> _green_brick_spot_(var);
    }

    std::getline(myfile, line);
    std::stringstream ss_blue(line);
    for (int var = 0; var < 5; ++var) {
      ss_blue >> _blue_brick_spot_(var);
    }

    ROS_INFO_STREAM("[PlanKeeper]: _red_brick_spot_" << _red_brick_spot_);
    ROS_INFO_STREAM("[PlanKeeper]: _green_brick_spot_" << _green_brick_spot_);
    ROS_INFO_STREAM("[PlanKeeper]: _blue_brick_spot_" << _blue_brick_spot_);
    ROS_INFO_STREAM("[PlanKeeper]: _wall_1_pos_" << _wall_1_pos_);
    ROS_INFO_STREAM("[PlanKeeper]: _wall_2_pos_" << _wall_2_pos_);
    ROS_INFO_STREAM("[PlanKeeper]: _wall_3_pos_" << _wall_3_pos_);
    ROS_INFO_STREAM("[PlanKeeper]: _wall_4_pos_" << _wall_4_pos_);

    this->calculateWallDefinition3DAndPlan();

    myfile.close();

    ROS_INFO_STREAM("[PlanKeeper]: Loaded backup plan with robot index " << robots_plans_indexes_[_robot_index_]);
    return true;

  } else {
    ROS_ERROR("[PlanKeeper]: Unable to open file: \"%s\"", filename.c_str());
    return false;
  };

}
//}

/* PlanKeeper::callbackChangeState() //{ */

bool PlanKeeper::callbackChangeState(plan_keeper::ChangeState::Request &req, plan_keeper::ChangeState::Response &res) {

  char tempStr[100];
  bool is_valid = req.state_id >= 0 && req.state_id < plan_keeper::PlanStateTypes::TYPE_COUNT;
  if (req.state_id != state_id_ && is_valid) {
    ROS_WARN("[PlanKeeper]: THIS_UAV: Switching states: %s -> %s", state_names_[state_id_].c_str(), state_names_[req.state_id].c_str());
    sprintf((char *)&tempStr, "Switching states: %s -> %s", state_names_[state_id_].c_str(), state_names_[req.state_id].c_str());
    state_id_ = req.state_id;

    res.message = tempStr;
    res.success = true;

    publishDiagnostics();

  } else if (!is_valid) {

    sprintf((char *)&tempStr, "Request state_id: %d is not valid. Should be in interval < 0, %d >.", req.state_id, plan_keeper::PlanStateTypes::TYPE_COUNT - 1);

    res.message = tempStr;
    res.success = false;

  } else {

    sprintf((char *)&tempStr, "State machine is already in state: %s", state_names_[state_id_].c_str());

    res.message = tempStr;
    res.success = true;
  }

  return true;
}

//}

/* MAIN FUNCTION //{ */
int main(int argc, char **argv) {
  // initialize ROS
  ros::init(argc, argv, "plan_keeper");
  ROS_INFO("[PlanKeeper]: Node initialized");

  // create class PlanKeeper
  PlanKeeper plan_keeper;

  int looprate                 = 20;
  int publish_arena_state_rate = 1;

  ros::Rate loop(looprate);
  double    count_publish = ((double)looprate) / ((double)publish_arena_state_rate);
  while (ros::ok()) {
    count_publish -= 1.0;
    if (count_publish <= 0) {
      count_publish = looprate / ((double)publish_arena_state_rate);
      plan_keeper.publish_arena_state();
    }

    ros::spinOnce();
    loop.sleep();
  }
  return EXIT_SUCCESS;
};

//}
