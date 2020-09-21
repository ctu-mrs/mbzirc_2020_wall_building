/* include header file of this class */
#include "arena_publisher.h"

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

namespace mbzirc_arena_config
{

/* onInit() //{ */

void ArenaPublisher::onInit() {

  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here

  /* obtain node handle */
  ros::NodeHandle nh("~");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */
  mrs_lib::ParamLoader param_loader(nh, "ArenaPublisher");

  param_loader.loadParam("uav_name", _uav_name_);

  param_loader.loadParam("static_tf_timer_rate", _rate_timer_static_tf_);

  // arena corners
  param_loader.loadParam("arena_ang_diff", arena_ang_diff_);
  param_loader.loadMatrixStatic("arena_center", arena_center_);
  param_loader.loadMatrixDynamic("arena_corners", arena_corners_, -1, 2);

  // safety area
  param_loader.loadMatrixDynamic("safety_area/safety_area", safety_area_, -1, 2);
  safety_polygon_ = std::make_shared<mrs_lib::Polygon>(safety_area_);

  // takeoff area
  param_loader.loadMatrixStatic("takeoff_center", takeoff_center_);
  param_loader.loadMatrixDynamic(TAKEOFF_ZONE_NAME, takeoff_zone_, -1, 2);
  takeoff_polygon_ = std::make_shared<mrs_lib::Polygon>(takeoff_zone_);

  std::string arena_type_tmp;
  param_loader.loadParam("arena_type", arena_type_tmp);

  if (arena_type_tmp == "ball_arena") {
    arena_type_ = BALL_ARENA;
  } else if (arena_type_tmp == "wall_arena") {
    arena_type_ = WALL_ARENA;
  } else if (arena_type_tmp == "fire_arena") {
    arena_type_ = FIRE_ARENA;
  } else if (arena_type_tmp == "default_arena") {
    arena_type_ = DEFAULT_ARENA;
  } else {
    ROS_ERROR(
        "[ArenaPublisher]: Wrong arena type specified in world file: %s \nCorrect arena types are: ball_arena, wall_arena, fire_arena \nFallback to: "
        "default_arena\nOnly safety_area and takeoff_zone loaded from world file, any arena zone can be added dynamically",
        arena_type_tmp.c_str());
    arena_type_ = DEFAULT_ARENA;
  }

  if (arena_type_ == BALL_ARENA) {

    /* ball arena //{ */

    // dropoff area
    param_loader.loadMatrixStatic("dropoff_center", dropoff_center_);
    param_loader.loadMatrixDynamic(DROPOFF_ZONE_NAME, dropoff_zone_, -1, 2);
    dropoff_polygon_ = std::make_shared<mrs_lib::Polygon>(dropoff_zone_);

    //}
  }

  if (arena_type_ == WALL_ARENA) {

    /* wall arena //{ */

    // uav brick area
    param_loader.loadMatrixStatic("uav_brick_center", uav_brick_center_);
    param_loader.loadMatrixDynamic(UAV_BRICK_ZONE_NAME, uav_brick_zone_, -1, 2);
    uav_brick_polygon_ = std::make_shared<mrs_lib::Polygon>(uav_brick_zone_);

    // uav wall area
    param_loader.loadMatrixStatic("uav_wall_center", uav_wall_center_);
    param_loader.loadMatrixDynamic(UAV_WALL_ZONE_NAME, uav_wall_zone_, -1, 2);
    uav_wall_polygon_ = std::make_shared<mrs_lib::Polygon>(uav_wall_zone_);

    // ugv brick area
    param_loader.loadMatrixStatic("ugv_brick_center", ugv_brick_center_);
    param_loader.loadMatrixDynamic(UGV_BRICK_ZONE_NAME, ugv_brick_zone_, -1, 2);
    ugv_brick_polygon_ = std::make_shared<mrs_lib::Polygon>(ugv_brick_zone_);

    // ugv wall area
    param_loader.loadMatrixStatic("ugv_wall_center", ugv_wall_center_);
    param_loader.loadMatrixDynamic(UGV_WALL_ZONE_NAME, ugv_wall_zone_, -1, 2);
    ugv_wall_polygon_ = std::make_shared<mrs_lib::Polygon>(ugv_wall_zone_);

    //}
  }

  if (arena_type_ == FIRE_ARENA) {

    /* fire arena //{ */

    param_loader.loadParam("default_challenge_mode", _default_challenge_mode_);
    ROS_INFO("[ArenaPublisher]: Default challenge mode: %d", _default_challenge_mode_);

    param_loader.loadParam("default_floor", target_floor_);
    ROS_INFO("[ArenaPublisher]: Default target floor: %d", target_floor_);

    param_loader.loadParam("default_indoor", target_indoor_);
    ROS_INFO("[ArenaPublisher]: Default target floor: %d", target_indoor_);

    param_loader.loadParam("call_start_timer_period", _period_timer_call_start_);

    // ground floor outside
    param_loader.loadMatrixStatic("ground_floor_center", ground_floor_center_);
    param_loader.loadParam("ground_floor_floor", ground_floor_floor_);
    param_loader.loadParam("ground_floor_ceiling", ground_floor_ceiling_);
    param_loader.loadMatrixDynamic("ground_floor_outside_points", ground_floor_outside_zone_, -1, 2);
    ground_floor_outside_polygon_ = std::make_shared<mrs_lib::Polygon>(ground_floor_outside_zone_);

    // first floor outside
    param_loader.loadMatrixStatic("first_floor_center", first_floor_center_);
    param_loader.loadParam("first_floor_floor", first_floor_floor_);
    param_loader.loadParam("first_floor_ceiling", first_floor_ceiling_);
    param_loader.loadMatrixDynamic("first_floor_outside_points", first_floor_outside_zone_, -1, 2);
    first_floor_outside_polygon_ = std::make_shared<mrs_lib::Polygon>(first_floor_outside_zone_);

    // second floor outside
    param_loader.loadMatrixStatic("second_floor_center", second_floor_center_);
    param_loader.loadParam("second_floor_floor", second_floor_floor_);
    param_loader.loadParam("second_floor_ceiling", second_floor_ceiling_);
    param_loader.loadMatrixDynamic("second_floor_outside_points", second_floor_outside_zone_, -1, 2);
    second_floor_outside_polygon_ = std::make_shared<mrs_lib::Polygon>(second_floor_outside_zone_);

    // outdoor fires
    param_loader.loadMatrixDynamic("fires_outdoor", outdoor_fires_, -1, 3);

    // windows
    param_loader.loadMatrixDynamic("windows", windows_, -1, 3);

    //}
  }

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ArenaPublisher]: Could not load all non-optional parameters.Shutting down.");
    ros::shutdown();
  }

  /* prepare arena zones //{ */

  arena_parameters_.header.frame_id = _uav_name_ + "/gps_origin";

  /* arena corners //{ */

  for (int i = 0; i < arena_corners_.rows(); i++) {
    geometry_msgs::Point p;
    p.x = arena_corners_(i, 0);
    p.y = arena_corners_(i, 1);
    p.z = 0.0;
    arena_parameters_.arena_corners.push_back(p);
  }

  //}

  /* safety area //{ */

  std::vector<geometry_msgs::Point> safety_area_points;
  {
    std::scoped_lock lock(mutex_safety_polygon_);

    safety_area_points = safety_polygon_->getPointMessageVector(0);
  }

  for (size_t i = 0; i < safety_area_points.size(); i++) {
    arena_parameters_.safety_area.push_back(safety_area_points[i]);
  }

  //}

  /* takeoff area //{ */

  std::vector<geometry_msgs::Point> takeoff_area_points;
  {
    std::scoped_lock lock(mutex_takeoff_polygon_);

    takeoff_area_points = takeoff_polygon_->getPointMessageVector(0);
  }

  mbzirc_msgs::ArenaZone takeoff_zone;
  takeoff_zone.zone_type = TAKEOFF_ZONE_NAME;
  for (size_t i = 0; i < takeoff_area_points.size(); i++) {
    takeoff_zone.corners.push_back(takeoff_area_points[i]);
  }

  arena_parameters_.zones.push_back(takeoff_zone);
  addMarker(takeoff_zone);

  //}

  /* ball arena //{ */

  std::vector<geometry_msgs::Point> dropoff_area_points;

  if (arena_type_ == BALL_ARENA) {

    /* dropoff area //{ */

    {
      std::scoped_lock lock(mutex_dropoff_polygon_);

      dropoff_area_points = dropoff_polygon_->getPointMessageVector(0);
    }

    mbzirc_msgs::ArenaZone dropoff_zone;
    dropoff_zone.zone_type = DROPOFF_ZONE_NAME;
    for (size_t i = 0; i < dropoff_area_points.size(); i++) {
      dropoff_zone.corners.push_back(dropoff_area_points[i]);
    }

    arena_parameters_.zones.push_back(dropoff_zone);
    addMarker(dropoff_zone);


    //}
  }

  //}

  /* wall arena //{ */

  std::vector<geometry_msgs::Point> uav_brick_area_points;
  std::vector<geometry_msgs::Point> uav_wall_area_points;
  std::vector<geometry_msgs::Point> ugv_brick_area_points;
  std::vector<geometry_msgs::Point> ugv_wall_area_points;

  if (arena_type_ == WALL_ARENA) {

    /* uav_brick area //{ */

    {
      std::scoped_lock lock(mutex_uav_brick_polygon_);

      uav_brick_area_points = uav_brick_polygon_->getPointMessageVector(0);
    }

    mbzirc_msgs::ArenaZone uav_brick_zone;
    uav_brick_zone.zone_type = UAV_BRICK_ZONE_NAME;
    for (size_t i = 0; i < uav_brick_area_points.size(); i++) {
      uav_brick_zone.corners.push_back(uav_brick_area_points[i]);
    }

    arena_parameters_.zones.push_back(uav_brick_zone);
    addMarker(uav_brick_zone);

    //}

    /* uav_wall area //{ */

    {
      std::scoped_lock lock(mutex_uav_wall_polygon_);

      uav_wall_area_points = uav_wall_polygon_->getPointMessageVector(0);
    }

    mbzirc_msgs::ArenaZone uav_wall_zone;
    uav_wall_zone.zone_type = UAV_WALL_ZONE_NAME;
    for (size_t i = 0; i < uav_wall_area_points.size(); i++) {
      uav_wall_zone.corners.push_back(uav_wall_area_points[i]);
    }

    arena_parameters_.zones.push_back(uav_wall_zone);
    addMarker(uav_wall_zone);

    //}

    /* ugv_brick area //{ */

    {
      std::scoped_lock lock(mutex_ugv_brick_polygon_);

      ugv_brick_area_points = ugv_brick_polygon_->getPointMessageVector(0);
    }

    mbzirc_msgs::ArenaZone ugv_brick_zone;
    ugv_brick_zone.zone_type = UGV_BRICK_ZONE_NAME;
    for (size_t i = 0; i < ugv_brick_area_points.size(); i++) {
      ugv_brick_zone.corners.push_back(ugv_brick_area_points[i]);
    }

    arena_parameters_.zones.push_back(ugv_brick_zone);
    addMarker(ugv_brick_zone);

    //}

    /* ugv_wall area //{ */

    {
      std::scoped_lock lock(mutex_ugv_wall_polygon_);

      ugv_wall_area_points = ugv_wall_polygon_->getPointMessageVector(0);
    }

    mbzirc_msgs::ArenaZone ugv_wall_zone;
    ugv_wall_zone.zone_type = UGV_WALL_ZONE_NAME;
    for (size_t i = 0; i < ugv_wall_area_points.size(); i++) {
      ugv_wall_zone.corners.push_back(ugv_wall_area_points[i]);
    }

    arena_parameters_.zones.push_back(ugv_wall_zone);
    addMarker(ugv_wall_zone);

    //}
  }

  //}

  /* fire arena //{ */

  std::vector<geometry_msgs::Point> ground_floor_outside_points;
  std::vector<geometry_msgs::Point> first_floor_outside_points;
  std::vector<geometry_msgs::Point> second_floor_outside_points;

  if (arena_type_ == FIRE_ARENA) {

    /* ground floor area //{ */

    {
      std::scoped_lock lock(mutex_ground_floor_outside_polygon_);

      ground_floor_outside_points = ground_floor_outside_polygon_->getPointMessageVector(0);
    }

    mbzirc_msgs::ArenaZone ground_floor_outside_zone;
    ground_floor_outside_zone.zone_type = GROUND_FLOOR_ZONE_NAME;
    mbzirc_msgs::ArenaZone ground_ceiling_outside_zone;
    ground_ceiling_outside_zone.zone_type = GROUND_CEILING_ZONE_NAME;

    for (size_t i = 0; i < ground_floor_outside_points.size(); i++) {
      ground_floor_outside_points[i].z = ground_floor_floor_;
      ground_floor_outside_zone.corners.push_back(ground_floor_outside_points[i]);
      ground_floor_outside_points[i].z = ground_floor_ceiling_;
      ground_ceiling_outside_zone.corners.push_back(ground_floor_outside_points[i]);
    }

    arena_parameters_.zones.push_back(ground_floor_outside_zone);
    addMarker(ground_floor_outside_zone);

    arena_parameters_.zones.push_back(ground_ceiling_outside_zone);
    addMarker(ground_ceiling_outside_zone);

    addConnectingMarker(ground_floor_outside_zone, ground_ceiling_outside_zone);

    //}

    /* first floor area //{ */

    {
      std::scoped_lock lock(mutex_first_floor_outside_polygon_);

      first_floor_outside_points = first_floor_outside_polygon_->getPointMessageVector(0);
    }

    mbzirc_msgs::ArenaZone first_floor_outside_zone;
    first_floor_outside_zone.zone_type = FIRST_FLOOR_ZONE_NAME;
    mbzirc_msgs::ArenaZone first_ceiling_outside_zone;
    first_ceiling_outside_zone.zone_type = FIRST_CEILING_ZONE_NAME;
    for (size_t i = 0; i < first_floor_outside_points.size(); i++) {
      first_floor_outside_points[i].z = first_floor_floor_;
      first_floor_outside_zone.corners.push_back(first_floor_outside_points[i]);
      first_floor_outside_points[i].z = first_floor_ceiling_;
      first_ceiling_outside_zone.corners.push_back(first_floor_outside_points[i]);
    }

    arena_parameters_.zones.push_back(first_floor_outside_zone);
    addMarker(first_floor_outside_zone);

    arena_parameters_.zones.push_back(first_ceiling_outside_zone);
    addMarker(first_ceiling_outside_zone);

    addConnectingMarker(first_floor_outside_zone, first_ceiling_outside_zone);

    //}

    /* second floor area //{ */

    {
      std::scoped_lock lock(mutex_second_floor_outside_polygon_);

      second_floor_outside_points = second_floor_outside_polygon_->getPointMessageVector(0);
    }

    mbzirc_msgs::ArenaZone second_floor_outside_zone;
    second_floor_outside_zone.zone_type = SECOND_FLOOR_ZONE_NAME;
    mbzirc_msgs::ArenaZone second_ceiling_outside_zone;
    second_ceiling_outside_zone.zone_type = SECOND_CEILING_ZONE_NAME;
    for (size_t i = 0; i < second_floor_outside_points.size(); i++) {
      second_floor_outside_points[i].z = second_floor_floor_;
      second_floor_outside_zone.corners.push_back(second_floor_outside_points[i]);
      second_floor_outside_points[i].z = second_floor_ceiling_;
      second_ceiling_outside_zone.corners.push_back(second_floor_outside_points[i]);
    }

    arena_parameters_.zones.push_back(second_floor_outside_zone);
    addMarker(second_floor_outside_zone);

    arena_parameters_.zones.push_back(second_ceiling_outside_zone);
    addMarker(second_ceiling_outside_zone);

    addConnectingMarker(second_floor_outside_zone, second_ceiling_outside_zone);

    //}
  }

  //}

  //}

  /* prepare marker array //{ */

  // arena corners
  for (int i = 0; i < 4; i++) {
    visualization_msgs::Marker corner_marker;

    double h                         = 5.0;
    corner_marker.header.frame_id    = _uav_name_ + "/gps_origin";
    corner_marker.type               = visualization_msgs::Marker::CYLINDER;
    corner_marker.ns                 = "arena_corners";
    corner_marker.id                 = i;
    corner_marker.color.a            = 0.5;
    corner_marker.scale.x            = 1.0;
    corner_marker.scale.y            = 1.0;
    corner_marker.scale.z            = h;
    corner_marker.color.r            = 0.5;
    corner_marker.color.g            = 0.5;
    corner_marker.color.b            = 0.5;
    corner_marker.pose.position.x    = arena_corners_(i, 0);
    corner_marker.pose.position.y    = arena_corners_(i, 1);
    corner_marker.pose.position.z    = h / 2;
    corner_marker.pose.orientation.w = 1.0;

    marker_array_.markers.push_back(corner_marker);

    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = _uav_name_ + "/gps_origin";
    text_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.ns              = "arena_corners_text";
    text_marker.id              = i;
    text_marker.pose.position.x = arena_corners_(i, 0);
    text_marker.pose.position.y = arena_corners_(i, 1);
    text_marker.pose.position.z = 6.0;
    text_marker.color.a         = 1;
    text_marker.scale.z         = 0.2;
    text_marker.color.r         = 1;
    text_marker.color.g         = 0;
    text_marker.color.b         = 0;
    std::stringstream ss;
    ss << "id: " << i << "\nx: " << arena_corners_(i, 0) << "\ny: " << arena_corners_(i, 1) << "\nz: " << 6.0;
    text_marker.text = ss.str();

    marker_array_.markers.push_back(text_marker);
  }

  // safety area marker
  visualization_msgs::Marker safety_marker;

  safety_marker.header.frame_id = _uav_name_ + "/gps_origin";
  safety_marker.type            = visualization_msgs::Marker::LINE_LIST;
  safety_marker.ns              = "safety_area";
  safety_marker.color.a         = 0.5;
  safety_marker.scale.x         = 0.2;
  safety_marker.color.r         = 1;
  safety_marker.color.g         = 0;
  safety_marker.color.b         = 0;

  for (size_t i = 0; i < safety_area_points.size(); i++) {
    safety_marker.points.push_back(safety_area_points[i]);
    safety_marker.points.push_back(safety_area_points[(i + 1) % safety_area_points.size()]);
  }

  marker_array_.markers.push_back(safety_marker);

  /* // takeoff area marker */
  /* visualization_msgs::Marker takeoff_marker; */

  /* takeoff_marker.header.frame_id = _uav_name_ + "/gps_origin"; */
  /* takeoff_marker.type            = visualization_msgs::Marker::LINE_LIST; */
  /* takeoff_marker.ns              = TAKEOFF_ZONE_NAME; */
  /* takeoff_marker.color.a         = 1; */
  /* takeoff_marker.scale.x         = 0.2; */
  /* takeoff_marker.color.r         = 0; */
  /* takeoff_marker.color.g         = 1; */
  /* takeoff_marker.color.b         = 0; */

  /* for (size_t i = 0; i < takeoff_area_points.size(); i++) { */
  /*   takeoff_marker.points.push_back(takeoff_area_points[i]); */
  /*   takeoff_marker.points.push_back(takeoff_area_points[(i + 1) % takeoff_area_points.size()]); */
  /* } */

  /* marker_array_.markers.push_back(takeoff_marker); */


  /* /1* ball arena markers //{ *1/ */

  /* if (arena_type_ == BALL_ARENA) { */

  /*   /1* dropoff area marker //{ *1/ */

  /*   visualization_msgs::Marker dropoff_marker; */

  /*   dropoff_marker.header.frame_id = _uav_name_ + "/gps_origin"; */
  /*   dropoff_marker.type            = visualization_msgs::Marker::LINE_LIST; */
  /*   dropoff_marker.ns              = DROPOFF_ZONE_NAME; */
  /*   dropoff_marker.color.a         = 1; */
  /*   dropoff_marker.scale.x         = 0.2; */
  /*   dropoff_marker.color.r         = 0; */
  /*   dropoff_marker.color.g         = 0; */
  /*   dropoff_marker.color.b         = 1; */

  /*   for (size_t i = 0; i < dropoff_area_points.size(); i++) { */
  /*     dropoff_marker.points.push_back(dropoff_area_points[i]); */
  /*     dropoff_marker.points.push_back(dropoff_area_points[(i + 1) % dropoff_area_points.size()]); */
  /*   } */

  /*   marker_array_.markers.push_back(dropoff_marker); */
  /* } */

  /* //} */

  /* //} */

  /* /1* wall area markers //{ *1/ */

  /* if (arena_type_ == WALL_ARENA) { */

  /*   /1* uav brick area marker //{ *1/ */
  /*   visualization_msgs::Marker uav_brick_marker; */

  /*   uav_brick_marker.header.frame_id = _uav_name_ + "/gps_origin"; */
  /*   uav_brick_marker.type            = visualization_msgs::Marker::LINE_LIST; */
  /*   uav_brick_marker.ns              = UAV_BRICK_ZONE_NAME; */
  /*   uav_brick_marker.color.a         = 1; */
  /*   uav_brick_marker.scale.x         = 0.2; */
  /*   uav_brick_marker.color.r         = 0; */
  /*   uav_brick_marker.color.g         = 0; */
  /*   uav_brick_marker.color.b         = 1; */

  /*   for (size_t i = 0; i < uav_brick_area_points.size(); i++) { */
  /*     uav_brick_marker.points.push_back(uav_brick_area_points[i]); */
  /*     uav_brick_marker.points.push_back(uav_brick_area_points[(i + 1) % uav_brick_area_points.size()]); */
  /*   } */

  /*   marker_array_.markers.push_back(uav_brick_marker); */


  /*   //} */

  /*   /1* uav wall area marker //{ *1/ */
  /*   visualization_msgs::Marker uav_wall_marker; */

  /*   uav_wall_marker.header.frame_id = _uav_name_ + "/gps_origin"; */
  /*   uav_wall_marker.type            = visualization_msgs::Marker::LINE_LIST; */
  /*   uav_wall_marker.ns              = "uav_wall"; */
  /*   uav_wall_marker.color.a         = 1; */
  /*   uav_wall_marker.scale.x         = 0.2; */
  /*   uav_wall_marker.color.r         = 0; */
  /*   uav_wall_marker.color.g         = 0; */
  /*   uav_wall_marker.color.b         = 1; */

  /*   for (size_t i = 0; i < uav_wall_area_points.size(); i++) { */
  /*     uav_wall_marker.points.push_back(uav_wall_area_points[i]); */
  /*     uav_wall_marker.points.push_back(uav_wall_area_points[(i + 1) % uav_wall_area_points.size()]); */
  /*   } */

  /*   marker_array_.markers.push_back(uav_wall_marker); */

  /*   //} */

  /*   /1* ugv brick area marker //{ *1/ */
  /*   visualization_msgs::Marker ugv_brick_marker; */

  /*   ugv_brick_marker.header.frame_id = _uav_name_ + "/gps_origin"; */
  /*   ugv_brick_marker.type            = visualization_msgs::Marker::LINE_LIST; */
  /*   ugv_brick_marker.ns              = "ugv_brick"; */
  /*   ugv_brick_marker.color.a         = 1; */
  /*   ugv_brick_marker.scale.x         = 0.2; */
  /*   ugv_brick_marker.color.r         = 0; */
  /*   ugv_brick_marker.color.g         = 0; */
  /*   ugv_brick_marker.color.b         = 1; */

  /*   for (size_t i = 0; i < ugv_brick_area_points.size(); i++) { */
  /*     ugv_brick_marker.points.push_back(ugv_brick_area_points[i]); */
  /*     ugv_brick_marker.points.push_back(ugv_brick_area_points[(i + 1) % ugv_brick_area_points.size()]); */
  /*   } */

  /*   marker_array_.markers.push_back(ugv_brick_marker); */


  /*   //} */

  /*   /1* ugv wall area marker //{ *1/ */
  /*   visualization_msgs::Marker ugv_wall_marker; */

  /*   ugv_wall_marker.header.frame_id = _uav_name_ + "/gps_origin"; */
  /*   ugv_wall_marker.type            = visualization_msgs::Marker::LINE_LIST; */
  /*   ugv_wall_marker.ns              = "ugv_wall"; */
  /*   ugv_wall_marker.color.a         = 1; */
  /*   ugv_wall_marker.scale.x         = 0.2; */
  /*   ugv_wall_marker.color.r         = 0; */
  /*   ugv_wall_marker.color.g         = 0; */
  /*   ugv_wall_marker.color.b         = 1; */

  /*   for (size_t i = 0; i < ugv_wall_area_points.size(); i++) { */
  /*     ugv_wall_marker.points.push_back(ugv_wall_area_points[i]); */
  /*     ugv_wall_marker.points.push_back(ugv_wall_area_points[(i + 1) % ugv_wall_area_points.size()]); */
  /*   } */

  /*   marker_array_.markers.push_back(ugv_wall_marker); */

  /*   //} */
  /* } */

  /* //} */

  //}

  // | ------------------ initialize subscribers ----------------- |

  // | ------------------ initialize publishers ----------------- |

  pub_markers_          = nh.advertise<visualization_msgs::MarkerArray>("markers_out", 1, true);
  pub_arena_parameters_ = nh.advertise<mbzirc_msgs::MbzircArenaParameters>("arena_parameters_out", 1, true);

  if (arena_type_ == BALL_ARENA) {
    pub_dropoff_pose_ = nh.advertise<geometry_msgs::PoseStamped>("dropoff_pose_out", 1, true);
  }

  if (arena_type_ == FIRE_ARENA) {
    pub_challenge_mode_ = nh.advertise<std_msgs::Int8>("challenge_mode_out", 1, true);
    pub_target_floor_   = nh.advertise<std_msgs::Int8>("target_floor_out", 1, true);
    pub_target_indoor_  = nh.advertise<std_msgs::Bool>("target_indoor_out", 1, true);
    pub_outdoor_fires_  = nh.advertise<geometry_msgs::PoseArray>("outdoor_fires_out", 1, true);
    pub_windows_        = nh.advertise<geometry_msgs::PoseArray>("windows_out", 1, true);
  }

  // publisher for tf
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();

  // | -------------------- initialize timers ------------------- |
  timer_call_start_ = nh.createTimer(ros::Duration(_period_timer_call_start_), &ArenaPublisher::callbackTimerCallStart, this, false, false);
  timer_static_tf_  = nh.createTimer(ros::Rate(_rate_timer_static_tf_), &ArenaPublisher::callbackTimerStaticTf, this);

  // | --------------- initialize service servers --------------- |
  srv_server_start_challenge_ = nh.advertiseService("start_in", &ArenaPublisher::callbackStartChallenge, this);
  srv_server_change_floor_    = nh.advertiseService("change_floor_in", &ArenaPublisher::callbackChangeFloor, this);
  srv_server_set_indoor_      = nh.advertiseService("set_indoor_in", &ArenaPublisher::callbackSetIndoor, this);
  srv_server_add_arena_zone_  = nh.advertiseService("add_arena_zone_in", &ArenaPublisher::callbackAddArenaZone, this);

  // | --------------- initialize service clients --------------- |

  srv_client_start_challenge_ = nh.serviceClient<mrs_msgs::SetInt>("start_out");

  // | ------- publish arena parameters on a latched topic ------ |
  arena_parameters_.header.stamp = ros::Time::now();

  try {
    pub_arena_parameters_.publish(arena_parameters_);
    ROS_INFO_ONCE("[ArenaPublisher]: Arena parameters published for the first time on a latched topic: %s", pub_arena_parameters_.getTopic().c_str());
  }
  catch (...) {
    ROS_ERROR("[ArenaPublisher]: Exception caught during publishing topic %s.", pub_arena_parameters_.getTopic().c_str());
  }

  // | --------- publish marker array on a latched topic -------- |
  try {
    pub_markers_.publish(marker_array_);
    ROS_INFO_ONCE("[ArenaPublisher]: Arena marker array published for the first time on a latched topic: %s", pub_markers_.getTopic().c_str());
  }
  catch (...) {
    ROS_ERROR("[ArenaPublisher]: Exception caught during publishing topic %s.", pub_markers_.getTopic().c_str());
  }

  /* ball arena publish //{ */

  // | --------- publish dropoff pose on a latched topic -------- |
  if (arena_type_ == BALL_ARENA) {
    geometry_msgs::PoseStamped dropoff_pose;
    dropoff_pose.header.stamp       = ros::Time::now();
    dropoff_pose.header.frame_id    = _uav_name_ + "/gps_origin";
    dropoff_pose.pose.position.x    = dropoff_center_(0);
    dropoff_pose.pose.position.y    = dropoff_center_(1);
    dropoff_pose.pose.position.z    = 0.0;
    dropoff_pose.pose.orientation.x = 0.0;
    dropoff_pose.pose.orientation.y = 0.0;
    dropoff_pose.pose.orientation.z = 0.0;
    dropoff_pose.pose.orientation.w = 1.0;

    try {
      pub_dropoff_pose_.publish(dropoff_pose);
      ROS_INFO_ONCE("[ArenaPublisher]: Dropoff center pose published for the first time on a latched topic: %s", pub_dropoff_pose_.getTopic().c_str());
    }
    catch (...) {
      ROS_ERROR("[ArenaPublisher]: Exception caught during publishing topic %s.", pub_dropoff_pose_.getTopic().c_str());
    }
  }

  //}

  /* fire arena publish //{ */

  if (arena_type_ == FIRE_ARENA) {
    // Publish target floor
    std_msgs::Int8 target_floor_out;
    target_floor_out.data = target_floor_;
    try {
      pub_target_floor_.publish(target_floor_out);
      ROS_INFO_ONCE("[ArenaPublisher]: Target floor published for the first time on a latched topic: %s", pub_target_floor_.getTopic().c_str());
    }
    catch (...) {
      ROS_ERROR("[ArenaPublisher]: Exception caught during publishing topic %s.", pub_target_floor_.getTopic().c_str());
    }

    // Publish target indoor
    std_msgs::Bool target_indoor_out;
    target_indoor_out.data = target_indoor_;
    try {
      pub_target_indoor_.publish(target_indoor_out);
      ROS_INFO_ONCE("[ArenaPublisher]: Target indoor published for the first time on a latched topic: %s", pub_target_indoor_.getTopic().c_str());
    }
    catch (...) {
      ROS_ERROR("[ArenaPublisher]: Exception caught during publishing topic %s.", pub_target_indoor_.getTopic().c_str());
    }

    // Publish outdoor fires
    geometry_msgs::PoseArray outdoor_fires_msg;
    outdoor_fires_msg.header.stamp    = ros::Time::now();
    outdoor_fires_msg.header.frame_id = _uav_name_ + "/gps_origin";
    for (int i = 0; i < outdoor_fires_.rows(); i++) {
      geometry_msgs::Pose pose_tmp;
      pose_tmp.position.x    = outdoor_fires_(i, 0);
      pose_tmp.position.y    = outdoor_fires_(i, 1);
      pose_tmp.position.z    = outdoor_fires_(i, 2);
      pose_tmp.orientation.x = 0.0;
      pose_tmp.orientation.y = 0.0;
      pose_tmp.orientation.z = 0.0;
      pose_tmp.orientation.w = 1.0;
      outdoor_fires_msg.poses.push_back(pose_tmp);
    }

    try {
      pub_outdoor_fires_.publish(outdoor_fires_msg);
      ROS_INFO_ONCE("[ArenaPublisher]: Outdoor fires published for the first time on a latched topic: %s", pub_outdoor_fires_.getTopic().c_str());
    }
    catch (...) {
      ROS_ERROR("[ArenaPublisher]: Exception caught during publishing topic %s.", pub_outdoor_fires_.getTopic().c_str());
    }

    // Publish windows
    geometry_msgs::PoseArray windows_msg;
    windows_msg.header.stamp    = ros::Time::now();
    windows_msg.header.frame_id = _uav_name_ + "/gps_origin";
    for (int i = 0; i < windows_.rows(); i++) {
      geometry_msgs::Pose pose_tmp;
      pose_tmp.position.x    = windows_(i, 0);
      pose_tmp.position.y    = windows_(i, 1);
      pose_tmp.position.z    = windows_(i, 2);
      pose_tmp.orientation.x = 0.0;
      pose_tmp.orientation.y = 0.0;
      pose_tmp.orientation.z = 0.0;
      pose_tmp.orientation.w = 1.0;
      windows_msg.poses.push_back(pose_tmp);
    }

    try {
      pub_windows_.publish(windows_msg);
      ROS_INFO_ONCE("[ArenaPublisher]: Windows published for the first time on a latched topic: %s", pub_windows_.getTopic().c_str());
    }
    catch (...) {
      ROS_ERROR("[ArenaPublisher]: Exception caught during publishing topic %s.", pub_windows_.getTopic().c_str());
    }
  }

  //}

  // | ------------------ finish initialization ----------------- |
  ROS_INFO_ONCE("[ArenaPublisher]: initialized");

  is_initialized_ = true;
}  // namespace mbzirc_arena_config
//}

// | ---------------------- msg callbacks --------------------- |

// | --------------------- timer callbacks -------------------- |

/* callbackTimerCallStart() //{ */

void ArenaPublisher::callbackTimerCallStart([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  // Call start service of mission planner
  ROS_INFO("[ArenaPublisher]: Calling start challenge service.");
  mrs_msgs::SetInt srv_start;
  int              mode_tmp = mrs_lib::get_mutexed(mutex_challenge_mode_, challenge_mode_);
  srv_start.request.value   = mode_tmp;
  srv_client_start_challenge_.call(srv_start);
  if (srv_start.response.success) {
    ROS_INFO("[ArenaPublisher]: Start challenge service called successfully: %s", srv_start.response.message.c_str());
    timer_call_start_.stop();
  } else {
    ROS_INFO("[ArenaPublisher]: Start challenge service call failed: %s", srv_start.response.message.c_str());
  }
}

//}

/* callbackTimerStaticTf() //{ */

void ArenaPublisher::callbackTimerStaticTf([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  /* publish TFs //{ */

  /* gps_origin - arena_origin //{ */

  tf_arena_.header.stamp            = ros::Time::now();
  tf_arena_.header.frame_id         = _uav_name_ + "/gps_origin";
  tf_arena_.child_frame_id          = _uav_name_ + "/arena_origin";
  tf_arena_.transform.translation.x = arena_center_[0];
  tf_arena_.transform.translation.y = arena_center_[1];
  tf_arena_.transform.translation.z = 0.0;

  tf2::Quaternion q_tmp;
  q_tmp.setRPY(0.0, 0.0, arena_ang_diff_);
  geometry_msgs::Quaternion q_msg;
  q_msg = tf2::toMsg(q_tmp);

  tf_arena_.transform.rotation = q_msg;

  try {
    broadcaster_->sendTransform(tf_arena_);
  }
  catch (...) {
    ROS_ERROR("[ArenaPublisher]: Exception caught during publishing TF: %s - %s.", tf_arena_.child_frame_id.c_str(), tf_arena_.header.frame_id.c_str());
  }

  //}

  /* gps_origin - takeoff_center //{ */

  tf_takeoff_.header.stamp            = ros::Time::now();
  tf_takeoff_.header.frame_id         = _uav_name_ + "/gps_origin";
  tf_takeoff_.child_frame_id          = _uav_name_ + "/takeoff_center";
  tf_takeoff_.transform.translation.x = takeoff_center_[0];
  tf_takeoff_.transform.translation.y = takeoff_center_[1];
  tf_takeoff_.transform.translation.z = 0.0;

  tf_takeoff_.transform.rotation = q_msg;

  try {
    broadcaster_->sendTransform(tf_takeoff_);
  }
  catch (...) {
    ROS_ERROR("[ArenaPublisher]: Exception caught during publishing TF: %s - %s.", tf_takeoff_.child_frame_id.c_str(), tf_takeoff_.header.frame_id.c_str());
  }

  //}

  /* ball arena TFs //{ */

  if (arena_type_ == BALL_ARENA) {

    /* gps_origin - dropoff_center //{ */

    tf_dropoff_.header.stamp            = ros::Time::now();
    tf_dropoff_.header.frame_id         = _uav_name_ + "/gps_origin";
    tf_dropoff_.child_frame_id          = _uav_name_ + "/dropoff_center";
    tf_dropoff_.transform.translation.x = dropoff_center_[0];
    tf_dropoff_.transform.translation.y = dropoff_center_[1];
    tf_dropoff_.transform.translation.z = 0.0;

    tf_dropoff_.transform.rotation = q_msg;

    try {
      broadcaster_->sendTransform(tf_dropoff_);
    }
    catch (...) {
      ROS_ERROR("[ArenaPublisher]: Exception caught during publishing TF: %s - %s.", tf_dropoff_.child_frame_id.c_str(), tf_dropoff_.header.frame_id.c_str());
    }

    //}
  }

  //}

  /* fire arena TFs //{ */

  if (arena_type_ == FIRE_ARENA) {

    /* gps_origin - gf_floor_center //{ */

    tf_gf_f_.header.stamp            = ros::Time::now();
    tf_gf_f_.header.frame_id         = _uav_name_ + "/gps_origin";
    tf_gf_f_.child_frame_id          = _uav_name_ + "/gf_floor_center";
    tf_gf_f_.transform.translation.x = ground_floor_center_[0];
    tf_gf_f_.transform.translation.y = ground_floor_center_[1];
    tf_gf_f_.transform.translation.z = ground_floor_floor_;

    tf_gf_f_.transform.rotation = q_msg;

    try {
      broadcaster_->sendTransform(tf_gf_f_);
    }
    catch (...) {
      ROS_ERROR("[ArenaPublisher]: Exception caught during publishing TF: %s - %s.", tf_gf_f_.child_frame_id.c_str(), tf_gf_f_.header.frame_id.c_str());
    }

    //}

    /* gps_origin - gf_ceiling_center //{ */

    tf_gf_c_.header.stamp            = ros::Time::now();
    tf_gf_c_.header.frame_id         = _uav_name_ + "/gps_origin";
    tf_gf_c_.child_frame_id          = _uav_name_ + "/gf_ceiling_center";
    tf_gf_c_.transform.translation.x = ground_floor_center_[0];
    tf_gf_c_.transform.translation.y = ground_floor_center_[1];
    tf_gf_c_.transform.translation.z = ground_floor_ceiling_;

    tf_gf_c_.transform.rotation = q_msg;

    try {
      broadcaster_->sendTransform(tf_gf_c_);
    }
    catch (...) {
      ROS_ERROR("[ArenaPublisher]: Exception caught during publishing TF: %s - %s.", tf_gf_c_.child_frame_id.c_str(), tf_gf_c_.header.frame_id.c_str());
    }

    //}

    /* gps_origin - 1f_floor_center //{ */

    tf_1f_f_.header.stamp            = ros::Time::now();
    tf_1f_f_.header.frame_id         = _uav_name_ + "/gps_origin";
    tf_1f_f_.child_frame_id          = _uav_name_ + "/1f_floor_center";
    tf_1f_f_.transform.translation.x = first_floor_center_[0];
    tf_1f_f_.transform.translation.y = first_floor_center_[1];
    tf_1f_f_.transform.translation.z = first_floor_floor_;

    tf_1f_f_.transform.rotation = q_msg;

    try {
      broadcaster_->sendTransform(tf_1f_f_);
    }
    catch (...) {
      ROS_ERROR("[ArenaPublisher]: Exception caught during publishing TF: %s - %s.", tf_1f_f_.child_frame_id.c_str(), tf_1f_f_.header.frame_id.c_str());
    }

    //}

    /* gps_origin - 1f_ceiling_center //{ */

    tf_1f_c_.header.stamp            = ros::Time::now();
    tf_1f_c_.header.frame_id         = _uav_name_ + "/gps_origin";
    tf_1f_c_.child_frame_id          = _uav_name_ + "/1f_ceiling_center";
    tf_1f_c_.transform.translation.x = first_floor_center_[0];
    tf_1f_c_.transform.translation.y = first_floor_center_[1];
    tf_1f_c_.transform.translation.z = first_floor_ceiling_;

    tf_1f_c_.transform.rotation = q_msg;

    try {
      broadcaster_->sendTransform(tf_1f_c_);
    }
    catch (...) {
      ROS_ERROR("[ArenaPublisher]: Exception caught during publishing TF: %s - %s.", tf_1f_c_.child_frame_id.c_str(), tf_1f_c_.header.frame_id.c_str());
    }

    //}

    /* gps_origin - 2f_floor_center //{ */

    tf_2f_f_.header.stamp            = ros::Time::now();
    tf_2f_f_.header.frame_id         = _uav_name_ + "/gps_origin";
    tf_2f_f_.child_frame_id          = _uav_name_ + "/2f_floor_center";
    tf_2f_f_.transform.translation.x = first_floor_center_[0];
    tf_2f_f_.transform.translation.y = first_floor_center_[1];
    tf_2f_f_.transform.translation.z = second_floor_floor_;

    tf_2f_f_.transform.rotation = q_msg;

    try {
      broadcaster_->sendTransform(tf_2f_f_);
    }
    catch (...) {
      ROS_ERROR("[ArenaPublisher]: Exception caught during publishing TF: %s - %s.", tf_2f_f_.child_frame_id.c_str(), tf_2f_f_.header.frame_id.c_str());
    }

    //}

    /* gps_origin - 2f_ceiling_center //{ */

    tf_2f_c_.header.stamp            = ros::Time::now();
    tf_2f_c_.header.frame_id         = _uav_name_ + "/gps_origin";
    tf_2f_c_.child_frame_id          = _uav_name_ + "/2f_ceiling_center";
    tf_2f_c_.transform.translation.x = first_floor_center_[0];
    tf_2f_c_.transform.translation.y = first_floor_center_[1];
    tf_2f_c_.transform.translation.z = second_floor_ceiling_;

    tf_2f_c_.transform.rotation = q_msg;

    try {
      broadcaster_->sendTransform(tf_2f_c_);
    }
    catch (...) {
      ROS_ERROR("[ArenaPublisher]: Exception caught during publishing TF: %s - %s.", tf_2f_c_.child_frame_id.c_str(), tf_2f_c_.header.frame_id.c_str());
    }

    //}
  }

  //}

  //}

  ROS_INFO_ONCE("[ArenaPublisher]: Publishing stati TFs at %d Hz rate", _rate_timer_static_tf_);
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackStartChallenge() */

bool ArenaPublisher::callbackStartChallenge([[maybe_unused]] mrs_msgs::SetInt::Request& req, mrs_msgs::SetInt::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Arena publisher not initialized!";
    ROS_WARN("[ArenaPublisher]: Cannot start challenge. Arena publisher is not initialized yet.");
    return true;
  }

  // Challenge mode was already set
  if (received_challenge_mode_) {
    int mode_active = mrs_lib::get_mutexed(mutex_challenge_mode_, challenge_mode_);
    ROS_WARN("[ArenaPublisher]: Received mode %d, but mode %d was already set.", (int)req.value, mode_active);
    res.success = true;  // false?
    res.message = "Challenge mode was already set";
    return true;
  }

  // If value out of valid range, change to default
  int mode_tmp = req.value;
  if (mode_tmp < 0 || mode_tmp > 2) {
    ROS_WARN("[ArenaPublisher]: Invalid challenge mode requested. Valid values are: 0 (default), 1, 2. Using default value.");
    mode_tmp = _default_challenge_mode_;
  }

  ROS_INFO("[ArenaPublisher]: Challenge mode %d chosen", mode_tmp);

  // Publish challenge mode
  std_msgs::Int8 challenge_mode_out;
  challenge_mode_out.data = mode_tmp;
  try {
    pub_challenge_mode_.publish(challenge_mode_out);
    ROS_INFO_ONCE("[ArenaPublisher]: Challenge mode published on a latched topic: %s", pub_challenge_mode_.getTopic().c_str());
  }
  catch (...) {
    ROS_ERROR("[ArenaPublisher]: Exception caught during publishing topic %s.", pub_challenge_mode_.getTopic().c_str());
  }


  mrs_lib::set_mutexed(mutex_challenge_mode_, mode_tmp, challenge_mode_);
  received_challenge_mode_ = true;

  // Call start service of mission planner
  ROS_INFO("[ArenaPublisher]: Calling start challenge service.");
  bool             success = false;
  mrs_msgs::SetInt srv_start;
  srv_start.request.value = mode_tmp;
  srv_client_start_challenge_.call(srv_start);
  if (srv_start.response.success) {
    ROS_INFO("[ArenaPublisher]: Start challenge service called successfully: %s", srv_start.response.message.c_str());
    success = true;
  } else {
    ROS_INFO("[ArenaPublisher]: Start challenge service call failed: %s", srv_start.response.message.c_str());
    success = false;
    timer_call_start_.start();
  }


  if (success) {
    res.message = "Start challenge service propagatd successfully";
    res.success = true;
  } else {
    res.message = "Start challenge service propagation failed. Will keep retrying.";
    res.success = true;  // The call from autostart was successful, now it's our responsibility to call the start service
  }

  return true;
}

//}

/* //{ callbackChangeFloor() */

bool ArenaPublisher::callbackChangeFloor([[maybe_unused]] mrs_msgs::SetInt::Request& req, mrs_msgs::SetInt::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Arena publisher not initialized!";
    ROS_WARN("[ArenaPublisher]: Cannot change floor. Arena publisher is not initialized yet.");
    return true;
  }

  // If value out of valid range, change to default
  int floor_tmp = req.value;
  if (floor_tmp < 0 || floor_tmp > 2) {
    ROS_WARN("[ArenaPublisher]: Invalid floor requested. Valid values are: 0, 1, 2. Not changing floor.");
    res.success = false;
    res.message = "Invalid floor requested. Valid values are: 0, 1, 2. Not changing floor.";
    return true;
  }

  ROS_INFO("[ArenaPublisher]: Target floor %d chosen", floor_tmp);

  // Publish target floor
  std_msgs::Int8 target_floor_out;
  target_floor_out.data = floor_tmp;
  try {
    pub_target_floor_.publish(target_floor_out);
    ROS_INFO_ONCE("[ArenaPublisher]: Target floor published on a latched topic: %s", pub_target_floor_.getTopic().c_str());
  }
  catch (...) {
    ROS_ERROR("[ArenaPublisher]: Exception caught during publishing topic %s.", pub_target_floor_.getTopic().c_str());
    res.success = false;
    res.message = "Setting target floor failed.";
    return true;
  }

  mrs_lib::set_mutexed(mutex_target_floor_, floor_tmp, target_floor_);

  res.success = true;
  res.message = "Target floor changed successfully";
  return true;
}

//}

/* //{ callbackSetIndoor() */

bool ArenaPublisher::callbackSetIndoor([[maybe_unused]] std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Arena publisher not initialized!";
    ROS_WARN("[ArenaPublisher]: Cannot set indoor. Arena publisher is not initialized yet.");
    return true;
  }

  // If value out of valid range, change to default
  bool indoor_tmp = req.data;

  ROS_INFO("[ArenaPublisher]: Indoor %d chosen", indoor_tmp);

  // Publish indoor
  std_msgs::Bool indoor_out;
  indoor_out.data = indoor_tmp;
  try {
    pub_target_indoor_.publish(indoor_out);
    ROS_INFO_ONCE("[ArenaPublisher]: Indoor published on a latched topic: %s", pub_target_indoor_.getTopic().c_str());
  }
  catch (...) {
    ROS_ERROR("[ArenaPublisher]: Exception caught during publishing topic %s.", pub_target_indoor_.getTopic().c_str());
    res.success = false;
    res.message = "Setting target indoor failed.";
    return true;
  }

  mrs_lib::set_mutexed(mutex_target_indoor_, indoor_tmp, target_indoor_);

  res.success = true;
  res.message = "Target indoor changed successfully";
  return true;
}

//}

/* //{ callbackIsInSafetyArea() */

bool ArenaPublisher::callbackIsInSafetyArea([[maybe_unused]] mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Arena publisher not initialized!";
    ROS_WARN("[ArenaPublisher]: Cannot evaluate the point, nodelet is not initialized.");
    return true;
  }


  bool is_inside;
  {
    std::scoped_lock lock(mutex_safety_polygon_);

    is_inside = safety_polygon_->isPointInside(req.goal[0], req.goal[1]);
  }

  if (is_inside) {

    ROS_INFO("[ArenaPublisher]: The point is INSIDE the safety area.");

    res.success = true;
    res.message = "The point is INSIDE the safety area.";

  } else {

    ROS_WARN("[ArenaPublisher]: The point is OUTSIDE the safety area.");
    res.success = false;
    res.message = "The point is OUTSIDE the safety area";
  }

  return true;
}

//}

/* //{ callbackIsInTakeoffArea() */

bool ArenaPublisher::callbackIsInTakeoffArea([[maybe_unused]] mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Arena publisher not initialized!";
    ROS_WARN("[ArenaPublisher]: Cannot evaluate the point, nodelet is not initialized.");
    return true;
  }


  bool is_inside;
  {
    std::scoped_lock lock(mutex_takeoff_polygon_);

    is_inside = takeoff_polygon_->isPointInside(req.goal[0], req.goal[1]);
  }

  if (is_inside) {

    ROS_INFO("[ArenaPublisher]: The point is INSIDE the takeoff area.");

    res.success = true;
    res.message = "The point is INSIDE the takeoff area.";

  } else {

    ROS_WARN("[ArenaPublisher]: The point is OUTSIDE the takeoff area.");
    res.success = false;
    res.message = "The point is OUTSIDE the takeoff area";
  }

  return true;
}

//}

/* //{ callbackIsInDropoffArea() */

bool ArenaPublisher::callbackIsInDropoffArea([[maybe_unused]] mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Arena publisher not initialized!";
    ROS_WARN("[ArenaPublisher]: Cannot evaluate the point, nodelet is not initialized.");
    return true;
  }


  bool is_inside;
  {
    std::scoped_lock lock(mutex_dropoff_polygon_);

    is_inside = dropoff_polygon_->isPointInside(req.goal[0], req.goal[1]);
  }

  if (is_inside) {

    ROS_INFO("[ArenaPublisher]: The point is INSIDE the dropoff area.");

    res.success = true;
    res.message = "The point is INSIDE the dropoff area.";

  } else {

    ROS_WARN("[ArenaPublisher]: The point is OUTSIDE the dropoff area.");
    res.success = false;
    res.message = "The point is OUTSIDE the dropoff area";
  }

  return true;
}

//}

/* //{ callbackAddArenaZone() */

bool ArenaPublisher::callbackAddArenaZone([[maybe_unused]] mbzirc_msgs::ArenaZoneSrv::Request& req, mbzirc_msgs::ArenaZoneSrv::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Arena publisher not initialized!";
    ROS_WARN("[ArenaPublisher]: Cannot add arena zone, nodelet is not initialized.");
    return true;
  }

  mbzirc_msgs::ArenaZone new_arena_zone = req.arena_zone;

  auto arena_type = mrs_lib::get_mutexed(mutex_arena_type_, arena_type_);

  switch (arena_type) {

      /* BALL_ARENA //{ */

    case BALL_ARENA:

      if (new_arena_zone.zone_type == DROPOFF_ZONE_NAME) {

        arena_parameters_.zones.push_back(new_arena_zone);
        addMarker(new_arena_zone);

      } else {
        ROS_WARN("[ArenaPublisher]: Tried to add invalid zone %s to BALL_ARENA", new_arena_zone.zone_type.c_str());
        res.success = false;
        res.message = "Tried to add invalid zone to BALL_ARENA.";
        return true;
      }

      break;

      //}

      /* WALL_ARENA //{ */

    case WALL_ARENA:

      if (new_arena_zone.zone_type == UAV_BRICK_ZONE_NAME || new_arena_zone.zone_type == UAV_WALL_ZONE_NAME ||
          new_arena_zone.zone_type == UGV_BRICK_ZONE_NAME || new_arena_zone.zone_type == UGV_WALL_ZONE_NAME) {

        arena_parameters_.zones.push_back(new_arena_zone);
        addMarker(new_arena_zone);

      } else {
        ROS_WARN("[ArenaPublisher]: Tried to add invalid zone %s to WALL_ARENA", new_arena_zone.zone_type.c_str());
        res.success = false;
        res.message = "Tried to add invalid zone to WALL_ARENA.";
        return true;
      }

      break;

      //}

      /* FIRE_ARENA //{ */

    case FIRE_ARENA:
      ROS_WARN("[ArenaPublisher]: TODO: Not implemented yet.");
      break;

      //}

      /* DEFAULT_ARENA //{ */

    case DEFAULT_ARENA:
      arena_parameters_.zones.push_back(new_arena_zone);
      addMarker(new_arena_zone);
      break;

      //}

    default:
      ROS_WARN("[ArenaPublisher]: Arena initialized with invalid arena type. Assuming DEFAULT_ARENA");
      arena_parameters_.zones.push_back(new_arena_zone);
      addMarker(new_arena_zone);
      break;
  }

  ROS_INFO("[ArenaPublisher]: New arena zone: %s added to arena.", new_arena_zone.zone_type.c_str());

  // Publish arena parameters
  arena_parameters_.header.stamp = ros::Time::now();

  try {
    pub_arena_parameters_.publish(arena_parameters_);
    ROS_INFO("[ArenaPublisher]: Arena parameters published on a latched topic: %s", pub_arena_parameters_.getTopic().c_str());
  }
  catch (...) {
    ROS_ERROR("[ArenaPublisher]: Exception caught during publishing topic %s.", pub_arena_parameters_.getTopic().c_str());
  }

  try {
    pub_markers_.publish(marker_array_);
    ROS_INFO("[ArenaPublisher]: Arena marker array published on a latched topic: %s", pub_markers_.getTopic().c_str());
  }
  catch (...) {
    ROS_ERROR("[ArenaPublisher]: Exception caught during publishing topic %s.", pub_markers_.getTopic().c_str());
  }

  res.success = true;
  res.message = "New arena zone added to arena.";

  return true;
}

//}

// | -------------------- support functions ------------------- |

/* addMarker() //{ */

void ArenaPublisher::addMarker(const mbzirc_msgs::ArenaZone& arena_zone_in) {

  visualization_msgs::Marker new_marker;

  new_marker.header.frame_id = _uav_name_ + "/gps_origin";
  new_marker.type            = visualization_msgs::Marker::LINE_LIST;
  new_marker.ns              = arena_zone_in.zone_type;
  new_marker.color.a         = 0.5;
  new_marker.scale.x         = 0.2;

  if (arena_zone_in.zone_type == TAKEOFF_ZONE_NAME) {
    new_marker.color.r = _takeoff_zone_color_.r;
    new_marker.color.g = _takeoff_zone_color_.g;
    new_marker.color.b = _takeoff_zone_color_.b;
  } else if (arena_zone_in.zone_type == UAV_BRICK_ZONE_NAME) {
    new_marker.color.r = _dropoff_zone_color_.r;
    new_marker.color.g = _dropoff_zone_color_.g;
    new_marker.color.b = _dropoff_zone_color_.b;
  } else if (arena_zone_in.zone_type == UAV_BRICK_ZONE_NAME) {
    new_marker.color.r = _uav_brick_zone_color_.r;
    new_marker.color.g = _uav_brick_zone_color_.g;
    new_marker.color.b = _uav_brick_zone_color_.b;
  } else if (arena_zone_in.zone_type == UAV_WALL_ZONE_NAME) {
    new_marker.color.r = _uav_wall_zone_color_.r;
    new_marker.color.g = _uav_wall_zone_color_.g;
    new_marker.color.b = _uav_wall_zone_color_.b;
  } else if (arena_zone_in.zone_type == UGV_BRICK_ZONE_NAME) {
    new_marker.color.r = _ugv_brick_zone_color_.r;
    new_marker.color.g = _ugv_brick_zone_color_.g;
    new_marker.color.b = _ugv_brick_zone_color_.b;
  } else if (arena_zone_in.zone_type == UGV_WALL_ZONE_NAME) {
    new_marker.color.r = _ugv_wall_zone_color_.r;
    new_marker.color.g = _ugv_wall_zone_color_.g;
    new_marker.color.b = _ugv_wall_zone_color_.b;
  }

  for (size_t i = 0; i < arena_zone_in.corners.size(); i++) {
    new_marker.points.push_back(arena_zone_in.corners[i]);
    new_marker.points.push_back(arena_zone_in.corners[(i + 1) % arena_zone_in.corners.size()]);
  }

  {
    std::scoped_lock lock(mutex_marker_array_);

    marker_array_.markers.push_back(new_marker);
  }
}

//}

/* addConnectingMarker() //{ */

void ArenaPublisher::addConnectingMarker(const mbzirc_msgs::ArenaZone& arena_zone1_in, const mbzirc_msgs::ArenaZone& arena_zone2_in) {

  visualization_msgs::Marker new_marker;

  new_marker.header.frame_id = _uav_name_ + "/gps_origin";
  new_marker.type            = visualization_msgs::Marker::LINE_LIST;
  new_marker.ns              = "connecting_line";
  new_marker.id              = connecting_line_id_++;
  new_marker.color.a         = 0.5;
  new_marker.scale.x         = 0.2;

  if (arena_zone1_in.zone_type == TAKEOFF_ZONE_NAME) {
    new_marker.color.r = _takeoff_zone_color_.r;
    new_marker.color.g = _takeoff_zone_color_.g;
    new_marker.color.b = _takeoff_zone_color_.b;
  } else if (arena_zone1_in.zone_type == UAV_BRICK_ZONE_NAME) {
    new_marker.color.r = _dropoff_zone_color_.r;
    new_marker.color.g = _dropoff_zone_color_.g;
    new_marker.color.b = _dropoff_zone_color_.b;
  } else if (arena_zone1_in.zone_type == UAV_BRICK_ZONE_NAME) {
    new_marker.color.r = _uav_brick_zone_color_.r;
    new_marker.color.g = _uav_brick_zone_color_.g;
    new_marker.color.b = _uav_brick_zone_color_.b;
  } else if (arena_zone1_in.zone_type == UAV_WALL_ZONE_NAME) {
    new_marker.color.r = _uav_wall_zone_color_.r;
    new_marker.color.g = _uav_wall_zone_color_.g;
    new_marker.color.b = _uav_wall_zone_color_.b;
  } else if (arena_zone1_in.zone_type == UGV_BRICK_ZONE_NAME) {
    new_marker.color.r = _ugv_brick_zone_color_.r;
    new_marker.color.g = _ugv_brick_zone_color_.g;
    new_marker.color.b = _ugv_brick_zone_color_.b;
  } else if (arena_zone1_in.zone_type == UGV_WALL_ZONE_NAME) {
    new_marker.color.r = _ugv_wall_zone_color_.r;
    new_marker.color.g = _ugv_wall_zone_color_.g;
    new_marker.color.b = _ugv_wall_zone_color_.b;
  }

  for (size_t i = 0; i < arena_zone1_in.corners.size(); i++) {
    new_marker.points.push_back(arena_zone1_in.corners[i]);
    new_marker.points.push_back(arena_zone2_in.corners[i]);
  }

  {
    std::scoped_lock lock(mutex_marker_array_);

    marker_array_.markers.push_back(new_marker);
  }
}

//}

/* matrixToPoints() //{ */

std::vector<mrs_msgs::Reference> ArenaPublisher::matrixToPoints(const Eigen::MatrixXd& matrix) {
  std::vector<mrs_msgs::Reference> points;

  for (int i = 0; i < matrix.rows(); i++) {
    mrs_msgs::Reference point;
    point.position.x = matrix.row(i)(0);
    point.position.y = matrix.row(i)(1);
    point.position.z = matrix.row(i)(2);
    point.heading    = matrix.row(i)(3);
    points.push_back(point);
  }

  return points;
}

//}

/* offsetPoints() //{ */

void ArenaPublisher::offsetPoints(std::vector<mrs_msgs::Reference>& points, const Eigen::MatrixXd& offset) {

  for (size_t i = 0; i < points.size(); i++) {
    points.at(i).position.x += offset(0);
    points.at(i).position.y += offset(1);
    points.at(i).position.z += offset(2);
    points.at(i).heading += offset(3);
  }
}

//}

/* distance() //{ */

double ArenaPublisher::distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose) {

  return std::sqrt(std::pow(waypoint.position.x - pose.position.x, 2) + std::pow(waypoint.position.y - pose.position.y, 2) +
                   std::pow(waypoint.position.z - pose.position.z, 2));
}

//}

}  // namespace mbzirc_arena_config

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(mbzirc_arena_config::ArenaPublisher, nodelet::Nodelet);
