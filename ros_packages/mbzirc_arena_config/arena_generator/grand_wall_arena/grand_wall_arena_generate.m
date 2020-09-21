clear all;

addpath ('../')
addpath ('../include')
 
% Load input
grand_wall_arena_input

% Generate common arena parameters
arena_generate

grand_wall_arena_write

grand_wall_arena_plot
