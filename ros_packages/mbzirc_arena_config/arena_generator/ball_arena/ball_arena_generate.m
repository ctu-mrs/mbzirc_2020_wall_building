clear all;

addpath ('../')
addpath ('../include')
 
% Load input
ball_arena_input

% Generate common arena parameters
arena_generate

if strcmp(coordinate_system, 'LL')

  % Convert dropoff points to UTM
  for i=1:size(dropoff_in,2)
    [dropoff_in(1, i), dropoff_in(2, i)] = LLToUTM(dropoff_in(1, i), dropoff_in(2, i));
  end

end

% Calculate dropoff points in local coordinates
dropoff(1, :) = dropoff_in(1, :) - utm_origin_x;
dropoff(2, :) = dropoff_in(2, :) - utm_origin_y;

% Homogeneous coordinates
dropoff = [dropoff; ones(1, size(dropoff,2))];

% Dropoff diagonal lines from points
dropoff_line1 = cross(dropoff(:, 1), dropoff(:, 3));
dropoff_line2 = cross(dropoff(:, 2), dropoff(:, 4));

% Get dropoff center
dropoff_center = linesIntersection(dropoff_line1, dropoff_line2);

ball_arena_write

ball_arena_plot
