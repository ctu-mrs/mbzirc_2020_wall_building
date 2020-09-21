/*
 * DatasetLoaderWall.h
 *
 *  Created on: 4. 9. 2019
 *      Author: Robert Penicka
 */

#ifndef SRC_DATASETLOADER_WALL_MBZIRC_H_
#define SRC_DATASETLOADER_WALL_MBZIRC_H_
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <cstdio>
#include <string>
#include <vector>
#include <map>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 1, 5> vector5;
typedef Eigen::Matrix<double, 1, 4> vector4;
typedef Eigen::Matrix<double, 1, 3> vector3;

enum BrickType {
	NO_BRICK_TYPE = 0, RED_BRICK_TYPE = 1, GREEN_BRICK_TYPE = 2, BLUE_BRICK_TYPE = 3, ORANGE_BRICK_TYPE = 4, WALL_BRICK_TYPE = 5
};

enum BrickSeeWallHavingBrickType {
	BRICK_SEE_WALL_HAVING_RED = 9, BRICK_SEE_WALL_HAVING_GREEN = 10, BRICK_SEE_WALL_HAVING_BLUE = 11
};

typedef std::vector<BrickType> BrickLayer;
typedef std::map<int, BrickLayer> BrickChannel;
typedef std::map<int, BrickChannel> WallDefinition;

typedef struct Brick3D {

	Brick3D() {
		x = NAN;
		y = NAN;
		z = NAN;
		yaw = NAN;
		id = 0;
		type = NO_BRICK_TYPE;
		layer = 0;
		channel = 0;
		wall_x = NAN;
		wall_y = NAN;
		wall_z = NAN;
		wall_yaw = NAN;
	}

	Brick3D(int id_, BrickType type_, int layer_, int channel_, double x_, double y_, double z_, double yaw_,double wall_x_, double wall_y_, double wall_z_, double wall_yaw_) {
		x = x_;
		y = y_;
		z = z_;
		yaw = yaw_;
		id = id_;
		type = type_;
		layer = layer_;
		channel = channel_;
		wall_x = wall_x_;
		wall_y = wall_y_;
		wall_z = wall_z_;
		wall_yaw = wall_yaw_;
	}

	double x;
	double y;
	double z;
	double yaw;
	int id;
	BrickType type;
	int layer;
	int channel;
	double wall_x;
	double wall_y;
	double wall_z;
	double wall_yaw;
} Brick3D;


typedef std::vector<Brick3D> BrickLayer3D;
typedef std::map<int, BrickLayer3D> BrickChannel3D;
typedef std::map<int, BrickChannel3D> WallDefinition3D;


class DatasetLoaderWallMBZIRC {
public:

	static std::vector<std::string> brick_names;
	static std::map<BrickType, std::vector<double>> brick_sizes;

	DatasetLoaderWallMBZIRC();
	virtual ~DatasetLoaderWallMBZIRC();
	static WallDefinition loadDataset(std::string filename);

	static WallDefinition3D wallDefinitionTo3DPopsitions(WallDefinition wall_definition,std::vector<vector5> wall_poses);
	static std::vector<std::string> split_string(std::string s, std::string delimiter);
	static std::string& trim(std::string &s, const char *t = " \t\n\r\f\v");
	static bool replace(std::string &str, const std::string &from, const std::string &to);
	static std::string getDefinitionString(WallDefinition definition);
	static std::string getDefinition3DString(WallDefinition3D definition) ;
}
;

#endif /* SRC_DATASETLOADER_WALL_MBZIRC_H_ */
