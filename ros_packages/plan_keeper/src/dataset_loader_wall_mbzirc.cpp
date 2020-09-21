/*
 * DatasetLoaderWall.cpp
 *
 *  Created on: 4. 9. 2019
 *      Author: Robert Penicka
 */

#include "dataset_loader_wall_mbzirc.h"
#include <ros/ros.h>

#define PRINT_DEF(str) ROS_INFO_STREAM( str );
#define PRINT_ERROR_DEF(str) ROS_ERROR_STREAM( str );

#define COMMENT_CHAR '#'

#define BLUE_BRICK_NAME "B"
#define RED_BRICK_NAME "R"
#define GREEN_BRICK_NAME "G"
#define ORANGE_BRICK_NAME "O"

#define LAYER_LEN (4.0)
#define BRICK_LAYER_LEN (3.6)
#define NUM_BRICKS_PER_LAYER (7.0)
#define BRICK_DISPLACEMENT ((LAYER_LEN - BRICK_LAYER_LEN)/NUM_BRICKS_PER_LAYER)
#define WALL_HEIGHT (1.7)

std::vector<std::string> DatasetLoaderWallMBZIRC::brick_names = { "NO_BRICK", "R", "G", "B", "O" };
/*
 * NO_BRICK_TYPE = 0, RED_BRICK_TYPE = 1, GREEN_BRICK_TYPE = 2, BLUE_BRICK_TYPE = 3, ORANGE_BRICK_TYPE = 4
 RED_BRICK: [30, 20, 20]
 GREEN_BRICK: [60, 20, 20]
 BLUE_BRICK: [120, 20, 20]
 ORANGE_BRICK: [180, 20, 20]
 */
std::map<BrickType, std::vector<double>> DatasetLoaderWallMBZIRC::brick_sizes = { { NO_BRICK_TYPE, { 0, 0, 0 } },
		{ RED_BRICK_TYPE, { 0.3, 0.2, 0.2 } }, { GREEN_BRICK_TYPE, { 0.6, 0.2, 0.2 } }, { BLUE_BRICK_TYPE, { 1.2, 0.2, 0.2 } }, { ORANGE_BRICK_TYPE, {
				1.8, 0.2, 0.2 } } };

DatasetLoaderWallMBZIRC::DatasetLoaderWallMBZIRC() {

}

DatasetLoaderWallMBZIRC::~DatasetLoaderWallMBZIRC() {

}

std::vector<std::string> DatasetLoaderWallMBZIRC::split_string(std::string s, std::string delimiter) {
	//INFO("tokenize:\""<<s<<"\" using delimiter :\""<<delimiter<<"\"")
	std::vector<std::string> tokens;
	size_t pos = 0;
	std::string token;
	while ((pos = s.find(delimiter)) != std::string::npos) {
		token = s.substr(0, pos);
		//INFO("add token \""<<token<<"\"")
		tokens.push_back(token);
		s.erase(0, pos + delimiter.length());
		//INFO("s after erase\""<<s<<"\"")
	}
	if (s.length() > 0) {
		//INFO("add token \""<<s<<"\"")
		tokens.push_back(s);
	}
	return tokens;
}

std::string& DatasetLoaderWallMBZIRC::trim(std::string &s, const char *t) {
	s.erase(0, s.find_first_not_of(t));
	s.erase(s.find_last_not_of(t) + 1);
	return s;
}

bool DatasetLoaderWallMBZIRC::replace(std::string &str, const std::string &from, const std::string &to) {
	size_t start_pos = str.find(from);
	if (start_pos == std::string::npos)
		return false;
	str.replace(start_pos, from.length(), to);
	return true;
}

WallDefinition DatasetLoaderWallMBZIRC::loadDataset(std::string filename) {
	PRINT_DEF("DatasetLoaderWallMBZIRC::loadDataset");

	WallDefinition definition;

	//example line: "Channel 1, Layer 1: BLUE GREEN RED RED RED RED GREEN"
	std::string delimiter_bricks = " ";

	std::ifstream in(filename.c_str(), std::ifstream::in);
	PRINT_DEF("reading problem file from "<<filename);
	int loaded = 0;
	if (!in) {
		std::cerr << "Cannot open " << filename << std::endl;
		exit(1);
	} else {
		std::string line;
		unsigned int lineNumber = 0;
		unsigned int actualGNID = 0;

		std::string trimmed_line;
		int layer = 2;
		while (getline(in, line)) {
			lineNumber++;

			trimmed_line = trim(line);

			std::vector<std::string> bricks = split_string(trimmed_line, delimiter_bricks);

			if (bricks.size() != 21) {
				std::cerr << "badly parsed from " << trimmed_line << " by delimiter " << delimiter_bricks << " with len " << bricks.size()
						<< " instead of 21" << std::endl;
			} else {
				//ales gute ja

				for (int var = 0; var < bricks.size(); ++var) {
					int channel = var / 7 + 1; // line contains all channels in one layer
					PRINT_DEF("channel is "<<channel<<" layer is "<<layer);
					if (bricks[var].find(RED_BRICK_NAME) != std::string::npos) {
						PRINT_DEF(bricks[var]<<" sholud be red brick");
						definition[channel][layer].push_back(RED_BRICK_TYPE);
					} else if (bricks[var].find(GREEN_BRICK_NAME) != std::string::npos) {
						PRINT_DEF(bricks[var]<<" sholud be green brick");
						definition[channel][layer].push_back(GREEN_BRICK_TYPE);
					} else if (bricks[var].find(BLUE_BRICK_NAME) != std::string::npos) {
						PRINT_DEF(bricks[var]<<" sholud be blue brick");
						definition[channel][layer].push_back(BLUE_BRICK_TYPE);
					} else if (bricks[var].find(ORANGE_BRICK_NAME) != std::string::npos) {
						PRINT_DEF(bricks[var]<<" sholud be orange brick");
						definition[channel][layer].push_back(ORANGE_BRICK_TYPE);
					}

				}

			}

			layer -= 1;
			if (layer <= 0) {
				PRINT_ERROR_DEF("layer is bellow 1"<<layer);
			}
		}
	}

	std::string definition_str = getDefinitionString(definition);
	PRINT_DEF(definition_str);
	PRINT_DEF("return loaded dataset")
	return definition;
}

WallDefinition3D DatasetLoaderWallMBZIRC::wallDefinitionTo3DPopsitions(WallDefinition definition, std::vector<vector5> wall_poses) {

	WallDefinition3D wall_xyz_yaw;
	WallDefinition::iterator it_ch;
	BrickChannel::iterator it_lay;
	int brick_id = 0;
	for (it_ch = definition.begin(); it_ch != definition.end(); it_ch++) {
		int channel = it_ch->first;

		vector5 channel_middle = wall_poses[channel - 1];
		double ch_yaw = wall_poses[channel - 1][3];

		for (it_lay = it_ch->second.begin(); it_lay != it_ch->second.end(); it_lay++) {
			int layer = it_lay->first;
			std::vector<BrickType> bricks = it_lay->second;

			double next_brick_start = -LAYER_LEN / 2.0 + BRICK_DISPLACEMENT / 2.0;

			for (int var = 0; var < bricks.size(); ++var) {
				BrickType brick_type = bricks[var];
				double brick_len = brick_sizes[brick_type][0];
				double brick_height = brick_sizes[brick_type][2];

				double pos_x = next_brick_start + brick_len / 2.0;
				next_brick_start += brick_len + BRICK_DISPLACEMENT;
				double pos_y = 0;
				double pos_z = WALL_HEIGHT + brick_height / 2.0 + (layer - 1) * brick_height;
				double pos_x_rot = pos_x * cos(ch_yaw) - pos_y * sin(ch_yaw) + channel_middle[0];
				double pos_y_rot = pos_x * sin(ch_yaw) + pos_y * cos(ch_yaw) + channel_middle[1];

				Brick3D brick;
				brick.id = brick_id;
				brick.type = brick_type;
				brick.x = pos_x_rot;
				brick.y = pos_y_rot;
				brick.z = pos_z;
				brick.yaw = ch_yaw;
				brick.channel = channel;
				brick.layer = layer;
				brick.wall_x = channel_middle[0];
				brick.wall_y = channel_middle[1];
				brick.wall_z = channel_middle[2];
				brick.wall_yaw = channel_middle[3];
				wall_xyz_yaw[channel][layer].push_back(brick);
				brick_id++;
			}
		}
	}

	return wall_xyz_yaw;
}

std::string DatasetLoaderWallMBZIRC::getDefinition3DString(WallDefinition3D definition) {
//example line: "Channel 1, Layer 1: BLUE GREEN RED RED RED RED GREEN"

	std::stringstream ss;
	WallDefinition3D::iterator it_ch;
	BrickChannel3D::iterator it_lay;
	for (it_ch = definition.begin(); it_ch != definition.end(); it_ch++) {
		int channel = it_ch->first;
		for (it_lay = it_ch->second.begin(); it_lay != it_ch->second.end(); it_lay++) {
			int layer = it_lay->first;
			BrickLayer3D bricks = it_lay->second;
			ss << "Channel " << channel << ", ";
			ss << "Layer " << layer << ":";
			for (int var = 0; var < bricks.size(); ++var) {
				Brick3D &brick = bricks[var];
				ss << " " << brick_names[bricks[var].type] << "(" << bricks[var].x << "," << bricks[var].y << ")";
			}
			ss << std::endl;
		}
	}
	return ss.str();
}

std::string DatasetLoaderWallMBZIRC::getDefinitionString(WallDefinition definition) {
//example line: "Channel 1, Layer 1: BLUE GREEN RED RED RED RED GREEN"

	std::stringstream ss;
	WallDefinition::iterator it_ch;
	BrickChannel::iterator it_lay;
	for (it_ch = definition.begin(); it_ch != definition.end(); it_ch++) {
		int channel = it_ch->first;
		for (it_lay = it_ch->second.begin(); it_lay != it_ch->second.end(); it_lay++) {
			int layer = it_lay->first;
			std::vector<BrickType> bricks = it_lay->second;
			ss << "Channel " << channel << ", ";
			ss << "Layer " << layer << ":";
			for (int var = 0; var < bricks.size(); ++var) {
				ss << " " << brick_names[bricks[var]];
			}
			ss << std::endl;
		}
	}
	return ss.str();
}

