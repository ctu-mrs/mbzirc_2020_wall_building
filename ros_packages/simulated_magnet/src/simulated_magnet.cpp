#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <string>
#include <vector>
#include <map>
#include <limits>
#include "mrs_msgs/GazeboAttach.h"
#include "mrs_msgs/GazeboAttachTyped.h"
#include <mbzirc_msgs/MagnetControl.h>
#include <mrs_msgs/GripperDiagnostics.h>
#include <std_msgs/String.h>
#include <algorithm>

//#include "MagnetControl.h"

using namespace std;
//#define M_PI	(3.14159265358979323846)  /* pi */
#define M_2PI (2 * M_PI)
#define sign_plus(x) ((x > 0) ? (1.0) : (-1.0))
struct State_2D;
struct LineSegment;
struct RepulsionLineSegment;
struct Object_Link;
struct DelayedConnection;

/*	params of movement	*/
double minX = -30;
double maxX = 30;
double minY = -50;
double maxY = 50;
double maxSpeedPerAxis = 1.0;
double maxAccPerAxis = 0.2;
double maxDiffAccelPerAxis = 0.1;
double timeStep = 0.1;
double outOfAreaAccMaxRatio = 0.2;
double maxConnectDistanceXY = 0.3;
double maxConnectDistanceXYLargeObject = 0.3;
double requiredRobotAboveDist = 0.5;
double requiredRobotAboveDistMaxErr = 0.04;
double startRepulsionFromDistance = 2.0;
double multiplyRepulseEffort = 10.0;
bool initially_magnet_on = false;
string turn_magnet_service_name = "turnOnMagnet";
string object_connected_topic_name = "objectConnected";
string stand_suffix = "stand";
vector<RepulsionLineSegment> repulse_line_segments;

// ros::ServiceClient client;
ros::Publisher modelPublisher;
ros::ServiceClient attachClient;
ros::ServiceClient detachClient;
ros::Subscriber subObjects;
vector<ros::ServiceServer> magnetServices;
vector<ros::Publisher> magnetConnectedPub;

// all objects
std::vector<std::string> all_objects;
std::vector<geometry_msgs::Pose> all_object_poses;
int num_uninitialized_objects = 0;
bool modelStatesCallbackReceived = false;

// robot objects
std::vector<std::string> robot_names;
std::map<std::string, bool> robot_attached;
std::map<std::string, Object_Link> robot_attached_object;
std::map<std::string, bool> robot_magnet_on;
std::vector<geometry_msgs::Pose> robotPoses;
std::vector<ros::Publisher> magnetDiagPublishers;

// delayed connection vector
std::vector<DelayedConnection> delayedConnetions;
double delay_attache_brick_to_ground;

/* STRUCTS //{ */

typedef struct Object_Link {
	Object_Link() {
		this->object = "";
		this->link = "";
	}
	Object_Link(string object_, string link_) {
		this->object = object_;
		this->link = link_;
	}
	string object;
	string link;
} Object_Link;

typedef struct DelayedConnection {
	Object_Link object1;
	Object_Link object2;
	std::string joint;
	ros::Time run_after;
} Connection;

typedef struct Point_2D {
	Point_2D() {
		x = 0;
		y = 0;
	}

	Point_2D(double x_, double y_) {
		x = x_;
		y = y_;
	}

	double distanceTo(Point_2D p) {
		return sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y));
	}

	Point_2D operator+(const Point_2D &p) const {
		return Point_2D(x + p.x, y + p.y);
	}

	Point_2D operator-(const Point_2D &p) const {
		return Point_2D(x - p.x, y - p.y);
	}

	Point_2D scalarProduct(double scalar) const {
		return Point_2D(x * scalar, y * scalar);
	}

	double x;
	double y;
} Point_2D;

std::ostream& operator<<(std::ostream &os, const Point_2D &obj) {
	os << "[" << obj.x << " " << obj.y << "]";
	return os;
}

typedef Point_2D Vector_2D;

typedef struct LineSegment {
	LineSegment(Point_2D a_, Point_2D b_) {
		a = a_;
		b = b_;
	}

	Point_2D a;
	Point_2D b;
} LineSegment;

typedef struct RepulsionLineSegment {
	RepulsionLineSegment(LineSegment lineSegment_, double repulsionx_, double repulsiony_) :
			lineSegment(lineSegment_) {
		repulsionx = repulsionx_;
		repulsiony = repulsiony_;
	}
	LineSegment lineSegment;
	double repulsionx;
	double repulsiony;
} RepulsionLineSegment;

typedef struct Line {
	double a;
	double b;
	double c;
	Line(double a_, double b_, double c_) {
		a = a_;
		b = b_;
		c = c_;
	}
	Line(LineSegment lineSegment) {
		initFromPoints(lineSegment.a, lineSegment.b);
	}

	Line(Vector_2D normalVector, Point_2D pointOnLine) {
		a = normalVector.x;
		b = normalVector.y;
		c = -(normalVector.x * pointOnLine.x + normalVector.y * pointOnLine.y);
	}

	Vector_2D normalVector() {
		return Vector_2D(a, b);
	}

	Point_2D intersect(Line perpLine) {
		double detAll = a * perpLine.b - b * perpLine.a;
		double detX = -c * perpLine.b - b * (-perpLine.c);
		double detY = a * (-perpLine.c) - (-c) * perpLine.a;
		return Point_2D(detX / detAll, detY / detAll);
	}

	void initFromPoints(Point_2D pointA, Point_2D pointB) {
		Vector_2D directionVec = pointB - pointA;
		a = -directionVec.y;
		b = directionVec.x;
		c = -(a * pointA.x + b * pointA.y);
	}
} Line;

typedef struct State_2D {
	State_2D(Point_2D position_, Point_2D speed_, Point_2D acceleration_) {
		position = position_;
		speed = speed_;
		acceleration = acceleration_;
	}
	State_2D() {
		position = Point_2D();
		speed = Point_2D();
		acceleration = Point_2D();
	}
	Point_2D position;
	Point_2D speed;
	Point_2D acceleration;
} State_2D;

typedef struct Point_2D Vector_2D;

//}

bool randSeeded = false;

double distanceFromLineSegments(Point_2D object_position, LineSegment lineSegment);

sig_atomic_t volatile g_request_shutdown = 0;

/* randDoubleMinMax() //{ */

double randDoubleMinMax(double min, double max) {

	if (!randSeeded) {
		srand(time(NULL));
		randSeeded = true;
	}
	double random = ((double) rand() / (double) RAND_MAX);
	random = min + random * (max - min);
	return random;
}

//}

/* randIntMinMax() //{ */

int randIntMinMax(int min, int max) {

	if (!randSeeded) {
		srand(time(NULL));
		randSeeded = true;
	}
	int random = min + (int) (((double) rand() / ((double) RAND_MAX + 1)) * (max - min + 1));
	return random;
}

//}

/* normalizeAngle() //{ */

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

//}

/* saturateState() //{ */

void saturateState(State_2D *objectState) {

	if (objectState->acceleration.x > maxAccPerAxis) {
		objectState->acceleration.x = maxAccPerAxis;
	} else if (objectState->acceleration.x < -maxAccPerAxis) {
		objectState->acceleration.x = -maxAccPerAxis;
	}

	if (objectState->acceleration.y > maxAccPerAxis) {
		objectState->acceleration.y = maxAccPerAxis;
	} else if (objectState->acceleration.y < -maxAccPerAxis) {
		objectState->acceleration.y = -maxAccPerAxis;
	}

	if (objectState->speed.x > maxSpeedPerAxis) {
		objectState->speed.x = maxSpeedPerAxis;
	} else if (objectState->speed.x < -maxSpeedPerAxis) {
		objectState->speed.x = -maxSpeedPerAxis;
	}

	if (objectState->speed.y > maxSpeedPerAxis) {
		objectState->speed.y = maxSpeedPerAxis;
	} else if (objectState->speed.y < -maxSpeedPerAxis) {
		objectState->speed.y = -maxSpeedPerAxis;
	}
}

//}

/* mySigIntHandler() //{ */

void mySigIntHandler([[maybe_unused]] int sig) {
	g_request_shutdown = 1;
}

//}

/* shutdownCallback() //{ */

void shutdownCallback(XmlRpc::XmlRpcValue &params, XmlRpc::XmlRpcValue &result) {

	int num_params = 0;
	if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
		num_params = params.size();
	if (num_params > 1) {
		std::string reason = params[1];
		ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
		g_request_shutdown = 1;  // Set flag
	}

	result = ros::xmlrpc::responseInt(1, "", 0);
}

//}

/* model_states_callback() //{ */

void model_states_callback(const gazebo_msgs::ModelStates modelStates) {

	vector<string> names = modelStates.name;
	vector<geometry_msgs::Pose> poses = modelStates.pose;
	vector<geometry_msgs::Twist> twists = modelStates.twist;
	// ROS_INFO_STREAM("model_states_callback with objects:");
	for (unsigned int receivedNamesId = 0; receivedNamesId < names.size(); ++receivedNamesId) {
		string objectName = names[receivedNamesId];
		// ROS_INFO_STREAM(objectName);

		// get positions of robots
		for (unsigned int var = 0; var < robot_names.size(); ++var) {
			if (objectName.compare(robot_names[var]) == 0) {
				robotPoses[var] = poses[receivedNamesId];
				/* ROS_INFO_STREAM("set position of "<< robot_names[var]); */
			}
		}

		for (unsigned int var = 0; var < all_objects.size(); ++var) {
			if (objectName.compare(all_objects[var]) == 0) {
				all_object_poses[var] = poses[receivedNamesId];
				/* ROS_INFO_STREAM("set position of "<< all_object_poses[var]); */
			}
		}
	}

	modelStatesCallbackReceived = true;
}

//}

/* getCloseObject() //{ */

map<string, Object_Link> getCloseObject() {

	// key is robot, value is object
	// ROS_INFO_STREAM("getCloseObject");
	double minDist = std::numeric_limits<double>::max();
	string minObject = "";
	string minRobot = "";
	map<string, Object_Link> closeObjects;

	for (unsigned int robotNameId = 0; robotNameId < robot_names.size(); ++robotNameId) {
		for (unsigned int objectNameId = 0; objectNameId < all_objects.size(); ++objectNameId) {

			geometry_msgs::Point robotPosition = robotPoses[robotNameId].position;
			geometry_msgs::Point objectPosition = all_object_poses[objectNameId].position;
			string robotName = robot_names[robotNameId];
			string objectName = all_objects[objectNameId];
			double xyDist = 0;
			double robotAboveZ = robotPosition.z - objectPosition.z;

			// distance is from point
			xyDist = sqrt(
					(objectPosition.x - robotPosition.x) * (objectPosition.x - robotPosition.x)
							+ (objectPosition.y - robotPosition.y) * (objectPosition.y - robotPosition.y));
			// test distances for normal object
			Object_Link obejctLink;
			obejctLink.object = objectName;
			obejctLink.link = "link";
			if (xyDist < maxConnectDistanceXY && fabs(robotAboveZ - requiredRobotAboveDist) < requiredRobotAboveDistMaxErr) {
				closeObjects[robotName] = obejctLink;
			}
			// calc minimal distant object
			if (xyDist + robotPosition.z < minDist) {
				minDist = xyDist + robotPosition.z;
				minRobot = robot_names[robotNameId];
				minObject = all_objects[objectNameId];
				// ROS_INFO_STREAM("closer "<<robot_names[robotNameId]<<" to "<<all_objects[objectNameId]);
			}
		}
	}
	/*
	 if (minDist != std::numeric_limits<double>::max()) {
	 ROS_INFO_STREAM("min dist: "<<minDist<<" between "<<minRobot<<" "<<minObject);
	 }
	 */
	return closeObjects;
}

//}

/* attach() //{ */

void attach(string robot_name, Object_Link object, string joint_type) {

	//ROS_INFO_STREAM("want to attach " << robot_name << " to " << object.object);
	// attach if magnet no
	if (robot_magnet_on[robot_name]) {
		//ROS_INFO_STREAM("robot " << robot_name << " magnet is on ");
		// attach if robot not attached
		if (!robot_attached[robot_name]) {
			int robot_id = -1;
			// git robot id

			for (unsigned int var = 0; var < robot_names.size(); ++var) {
				if (robot_names[var].compare(robot_name) == 0) {
					robot_id = var;
					break;
				}
			}

			if (robot_id != -1) {

				// detach from stand
				mrs_msgs::GazeboAttach detachFromStandCall;
				detachFromStandCall.request.model_name_1 = object.object + stand_suffix;
				detachFromStandCall.request.link_name_1 = "link";
				detachFromStandCall.request.model_name_2 = object.object;
				detachFromStandCall.request.link_name_2 = "link";

				// TODO check whether the detach was successfull
				// detachClient.call(detachFromStandCall);

				// TODO add some delay, e.g. 100ms

				mrs_msgs::GazeboAttachTyped atachCall;
				atachCall.request.model_name_1 = robot_name;
				atachCall.request.link_name_1 = "base_link";
				atachCall.request.model_name_2 = object.object;
				atachCall.request.link_name_2 = object.link;
				atachCall.request.joint_type = joint_type;
				ROS_INFO_STREAM(
						"attach " << atachCall.request.model_name_1 << " link " << atachCall.request.link_name_1 << " to "
								<< atachCall.request.model_name_2 << " link " << atachCall.request.link_name_2);

				if (!attachClient.call(atachCall)) {
					ROS_INFO_STREAM(
							"can not attach " << atachCall.request.model_name_1 << " link " << atachCall.request.link_name_1 << " to "
									<< atachCall.request.model_name_2 << " link " << atachCall.request.link_name_2 << " response:"
									<< atachCall.response.ok);
				} else {

					robot_attached[robot_name] = true;
					robot_attached_object[robot_name] = object;
					ROS_INFO_STREAM("attached");
					std_msgs::String message;
					message.data = object.object;
					magnetConnectedPub[robot_id].publish(message);
				}
			} else {
				ROS_ERROR_STREAM("attach callback " << robot_name << " link " << "base_link" << " to " << object.object << " link " << object.link);
				ROS_ERROR_STREAM("can not find robot name");
			}
		}
	}
}

//}

/* detachAll() //{ */

void detachAll(string robot) {

	// detach if magnet is off
	if (!robot_magnet_on[robot]) {
		// detach if robot atached
		if (robot_attached[robot]) {
			mrs_msgs::GazeboAttach detachCall;
			detachCall.request.model_name_1 = robot;
			detachCall.request.link_name_1 = "base_link";
			detachCall.request.model_name_2 = robot_attached_object[robot].object;
			detachCall.request.link_name_2 = robot_attached_object[robot].link;
			ROS_INFO_STREAM(
					"attach " << robot << " link " << detachCall.request.link_name_1 << " to " << robot_attached_object[robot].object << " link "
							<< detachCall.request.link_name_2);

			if (!detachClient.call(detachCall)) {
				ROS_INFO_STREAM(
						"can not attach " << robot << " link " << detachCall.request.link_name_1 << " to " << robot_attached_object[robot].object
								<< " link " << detachCall.request.link_name_2 << " response:" << detachCall.response.ok);
			} else {
				ROS_INFO_STREAM("detached " << robot_attached_object[robot].object);

				/* DelayedConnection delayed_connection; */
				/* delayed_connection.object1.object = robot_attached_object[robot].object; */
				/* delayed_connection.object1.link = robot_attached_object[robot].link; */
				/* delayed_connection.object2.object = "ground_plane"; */
				/* delayed_connection.object2.link = "link"; */
				/* delayed_connection.joint = "revolute"; */
				/* delayed_connection.run_after = ros::Time::now() + ros::Duration(delay_attache_brick_to_ground); */
				/* delayedConnetions.push_back(delayed_connection); */

				robot_attached[robot] = false;
				robot_attached_object[robot].object = "";
				robot_attached_object[robot].link = "";


				/*
				 mrs_msgs::GazeboAttachTyped atachCall;
				 atachCall.request.model_name_1 = robot;
				 atachCall.request.link_name_1  = "base_link";
				 atachCall.request.model_name_2 = "ground_plane";
				 atachCall.request.link_name_2  = "link";
				 atachCall.request.joint_type   = "revolute";
				 attachClient.call(atachCall);
				 */
			}
		}
	}
}

//}

/* testConnectObjects() //{ */

void testConnectObjects() {

	map<string, Object_Link> closeObjectPairs = getCloseObject();
	if (closeObjectPairs.size() > 0) {
		map<string, Object_Link>::iterator it;
		for (it = closeObjectPairs.begin(); it != closeObjectPairs.end(); it++) {
			// ball, revolute gearbox, prismatic, revolute2, universal, piston
			attach(it->first, it->second, "fixed");
		}
	}
	for (unsigned int var = 0; var < robot_names.size(); ++var) {
		mrs_msgs::GripperDiagnostics gripper_msg;
		gripper_msg.stamp = ros::Time::now();
		gripper_msg.gripper_on = robot_magnet_on[robot_names[var]];
		gripper_msg.gripping_object = robot_attached[robot_names[var]];
		magnetDiagPublishers[var].publish(gripper_msg);
	}
}

//}

/* testConnectObjects() //{ */

void testConnectDelayed() {
	for ( int var = delayedConnetions.size() - 1; var >= 0; --var) {
		ros::Duration dur_till_run = ros::Time::now() - delayedConnetions[var].run_after;
		if (dur_till_run.toSec() >= 0) {
			mrs_msgs::GazeboAttachTyped atachCall;
			atachCall.request.model_name_1 = delayedConnetions[var].object1.object;
			atachCall.request.link_name_1 = delayedConnetions[var].object1.link;
			atachCall.request.model_name_2 = delayedConnetions[var].object2.object;
			atachCall.request.link_name_2 = delayedConnetions[var].object2.link;
			atachCall.request.joint_type = delayedConnetions[var].joint;
			ROS_INFO_STREAM(
					"try attach delayed " << atachCall.request.model_name_1 << " link " << atachCall.request.link_name_1 << " to "
							<< atachCall.request.model_name_2 << " link " << atachCall.request.link_name_2);

			if (!attachClient.call(atachCall)) {
				ROS_INFO_STREAM(
						"can not attach " << atachCall.request.model_name_1 << " link " << atachCall.request.link_name_1 << " to "
								<< atachCall.request.model_name_2 << " link " << atachCall.request.link_name_2 << " response:"
								<< atachCall.response.ok);
			} else {
				ROS_INFO_STREAM("attached");
				delayedConnetions.erase(delayedConnetions.begin()+var);
			}
		} else {
			ROS_INFO_STREAM("wait delayed " << dur_till_run.toSec());
		}
	}
}

//}

/* magnetTurnHandler() //{ */

bool magnetTurnHandler(ros::ServiceEvent<mbzirc_msgs::MagnetControl::Request, mbzirc_msgs::MagnetControl::Response> &event) {

	// ROS_INFO_STREAM("Dropping object of " << event.getCallerName());
	std::map<std::string, std::string>::iterator it;
	string serviceNameHeaderKey = "service";
	string serviceName = "";
	string handled_robot_name = "";

	for (it = event.getConnectionHeader().begin(); it != event.getConnectionHeader().end(); it++) {
		// ROS_INFO_STREAM("header{" << it->first<<":"<< it->second<<"}");
		if (it->first.compare(serviceNameHeaderKey) == 0) {
			serviceName = it->second;
			// ROS_INFO_STREAM("setting service name to " <<serviceName);
			for (unsigned int robotNameId = 0; robotNameId < robot_names.size(); ++robotNameId) {
				string robotName = robot_names[robotNameId];
				string magentServiceName = "/" + robotName + "/" + turn_magnet_service_name;
				if (serviceName.compare(magentServiceName) == 0) {
					handled_robot_name = robotName;
					// ROS_INFO_STREAM("setting handled_robot_name to " <<handled_robot_name);
				}
			}
		}
	}

	if (handled_robot_name.size() > 0) {
		if (robot_attached.count(handled_robot_name)) {
			if (event.getRequest().turnOnOff) {
				// turn on
				robot_magnet_on[handled_robot_name] = true;
				event.getResponse().stateAfter = true;

				ROS_INFO_STREAM("turn on magnet of " << handled_robot_name);
			} else {
				// turn off
				robot_magnet_on[handled_robot_name] = false;
				detachAll(handled_robot_name);
				event.getResponse().stateAfter = false;

				ROS_INFO_STREAM("turn off magnet of " << handled_robot_name);
			}
		} else {
			event.getResponse().stateAfter = false;
		}
	} else {
		ROS_ERROR_STREAM("can not find robot name inside service name " << serviceName);
		event.getResponse().stateAfter = false;
		return true;
	}
	return true;
}

//}

/* main() //{ */

int main(int argc, char **argv) {

	// Override SIGINT handler
	ros::init(argc, argv, "simulated_magnet", ros::init_options::NoSigintHandler);
	signal(SIGINT, mySigIntHandler);

	// Override XMLRPC shutdown
	ros::XMLRPCManager::instance()->unbind("shutdown");
	ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

	ros::init(argc, argv, "simulated_magnet_node");
	ros::NodeHandle nh_ = ros::NodeHandle("~");
	// vector<string> paramNames();
	// nh_.getParamNames(paramNames);
	all_objects = nh_.param("all_objects", std::vector<std::string>());
	minX = nh_.param("minX", -30);
	maxX = nh_.param("maxX", 30);
	minY = nh_.param("minY", -50);
	maxY = nh_.param("maxY", 50);
	initially_magnet_on = nh_.param("initially_magnet_on", false);
	maxSpeedPerAxis = nh_.param("maxSpeedPerAxis", 1.0);
	maxAccPerAxis = nh_.param("maxAccPerAxis", 0.2);
	maxDiffAccelPerAxis = nh_.param("maxDiffAccelPerAxis", 0.1);
	outOfAreaAccMaxRatio = nh_.param("outOfAreaAccMaxRatio", 0.2);
	timeStep = nh_.param("timeStep", 0.1);
	maxConnectDistanceXY = nh_.param("maxConnectDistanceXY", 0.3);
	maxConnectDistanceXYLargeObject = nh_.param("maxConnectDistanceXYLargeObject", 0.3);
	requiredRobotAboveDistMaxErr = nh_.param("requiredRobotAboveDistMaxErr", 0.1);
	vector<double> repulse_lines = nh_.param("repulse_lines", vector<double>());
	startRepulsionFromDistance = nh_.param("startRepulsionFromDistance", 2.0);
	multiplyRepulseEffort = nh_.param("multiplyRepulseEffort", 10.0);
	stand_suffix = nh_.param("stand_suffix", string("stand"));

	delay_attache_brick_to_ground = nh_.param("delay_attache_brick_to_ground",0.1);

	all_object_poses.resize(all_objects.size());
	robot_names = nh_.param("robot_names", std::vector<std::string>());
	robotPoses.resize(robot_names.size());
	magnetDiagPublishers.resize(robot_names.size());
	magnetServices.resize(robot_names.size());
	magnetConnectedPub.resize(robot_names.size());

	for (unsigned int var = 0; var < robot_names.size(); ++var) {
		robot_attached[robot_names[var]] = false;
		robot_magnet_on[robot_names[var]] = initially_magnet_on;
		ROS_INFO_STREAM("set robot " << robot_names[var] << " magnet to " << initially_magnet_on);
		robot_attached_object[robot_names[var]].object = "";
		robot_attached_object[robot_names[var]].link = "";
		robotPoses[var].position.x = std::numeric_limits<double>::max();
		robotPoses[var].position.y = std::numeric_limits<double>::max();
		robotPoses[var].position.z = std::numeric_limits<double>::max();
		magnetDiagPublishers[var] = nh_.advertise < mrs_msgs::GripperDiagnostics > ("/" + robot_names[var] + "/mrs_gripper/gripper_diagnostics", 1);
	}

	ROS_INFO_STREAM("init model state service client");
	// client = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	// gazebo/set_model_state
	modelPublisher = nh_.advertise < gazebo_msgs::ModelState > ("/gazebo/set_model_state", 100);
	ROS_INFO_STREAM("init detach attach service clients");
	attachClient = nh_.serviceClient < mrs_msgs::GazeboAttachTyped > ("/link_attacher_node/attach_typed");
	detachClient = nh_.serviceClient < mrs_msgs::GazeboAttach > ("/link_attacher_node/detach");
	ROS_INFO_STREAM("init model states subscriber");
	subObjects = nh_.subscribe("/gazebo/model_states", 1, model_states_callback);

	for (unsigned int var = 0; var < robot_names.size(); ++var) {
		string robotName = robot_names[var];
		string magentServiceName = "/" + robotName + "/" + turn_magnet_service_name;
		string objectServiceName = "/" + robotName + "/" + object_connected_topic_name;
		magnetServices[var] = nh_.advertiseService(magentServiceName, magnetTurnHandler);
		magnetConnectedPub[var] = nh_.advertise < std_msgs::String > (objectServiceName, 1);
	}

	while (!g_request_shutdown && num_uninitialized_objects > 0) {
		ros::spinOnce();
		ROS_INFO_STREAM("waiting for objects postions " << num_uninitialized_objects);
		if (modelStatesCallbackReceived && num_uninitialized_objects > 0) {
			ROS_INFO_STREAM("not all objects exists in the world");
			ROS_INFO_STREAM("exiting");
			ros::Rate(0.1).sleep();
			g_request_shutdown = 1;
			break;
		}
		ros::Rate(1.0 / timeStep).sleep();  // sleep
	}

	/*
	// Position
	geometry_msgs::Point pr2_position;
	pr2_position.x = 0.0;
	pr2_position.y = 0.0;
	pr2_position.z = 0.0;
	// orientation
	geometry_msgs::Quaternion pr2_orientation;
	pr2_orientation.x = 0.0;
	pr2_orientation.y = 0.0;
	pr2_orientation.z = 0.0;
	pr2_orientation.w = 1.0;

	
	ROS_INFO_STREAM("running object movement with objects:");

	for (unsigned int var = 0; var < all_objects.size(); ++var) {
		ROS_INFO_STREAM("attach stand with objects: " << all_objects[var] + stand_suffix << " with " << all_objects[var]);
		mrs_msgs::GazeboAttachTyped atachCall;
		atachCall.request.model_name_1 = all_objects[var] + stand_suffix;
		atachCall.request.link_name_1 = "link";
		atachCall.request.model_name_2 = all_objects[var];
		atachCall.request.link_name_2 = "link";
		atachCall.request.joint_type = "revolute";
		if (attachClient.call(atachCall)) {
			ROS_INFO_STREAM(all_objects[var] + stand_suffix << " with " << all_objects[var] << " attached");
		} else {
			ROS_INFO_STREAM("can not attach " << all_objects[var] + stand_suffix << " with " << all_objects[var] << " attached");
		}
	}
	*/

	ROS_INFO_STREAM("sleep one second after attaching to stands");
	ros::Duration(1.0).sleep();

	ROS_INFO_STREAM("now running.....");

	ros::Rate moveRate(1.0 / timeStep);

	while (!g_request_shutdown) {
		ros::spinOnce();
		ros::spinOnce();
		testConnectObjects();
		testConnectDelayed();
		moveRate.sleep();  // sleep
	}

	ros::shutdown();
}

//}
