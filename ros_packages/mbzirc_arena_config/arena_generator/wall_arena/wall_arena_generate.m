clear all;

addpath ('../')
addpath ('../include')
 
% Load input
wall_arena_input

% Generate common arena parameters
arena_generate

% Calculate corners of uav brick area
[uav_brick1, uav_brick2, uav_brick3, uav_brick4] = getRect(uav_brick_center, ang_diff, uav_brick_length, uav_brick_width);

% Calculate corners of uav wall area
[uav_wall1, uav_wall2, uav_wall3, uav_wall4] = getRect(uav_wall_center, ang_diff, uav_wall_length, uav_wall_width);

% Calculate corners of ugv brick area
[ugv_brick1, ugv_brick2, ugv_brick3, ugv_brick4] = getRect(ugv_brick_center, ang_diff, ugv_brick_length, ugv_brick_width);

% Calculate corners of ugv wall area
[ugv_wall1, ugv_wall2, ugv_wall3, ugv_wall4] = getRect(ugv_wall_center, ang_diff, ugv_wall_length, ugv_wall_width);

wall_arena_write

wall_arena_plot
