clear all;

addpath ('../')
addpath ('../include')
 
% Load input
fire_arena_input

% Generate common arena parameters
arena_generate

% %{ ground floor outside

if strcmp(coordinate_system, 'LL')

  % Convert points in to UTM
  for i=1:size(ground_floor_outside_points_in,2)
    [ground_floor_outside_points_in(1, i), ground_floor_outside_points_in(2, i)] = LLToUTM(ground_floor_outside_points_in(1, i), ground_floor_outside_points_in(2, i));
  end

end

% Calculate points in local coordinates
ground_floor_outside_points(1, :) = ground_floor_outside_points_in(1, :) - utm_origin_x;
ground_floor_outside_points(2, :) = ground_floor_outside_points_in(2, :) - utm_origin_y;

% Get center of building
pts = ground_floor_outside_points;
p1 = [pts(1,1), pts(2,1)]';
p2 = [pts(1,2), pts(2,2)]';
p3 = [pts(1,3), pts(2,3)]';
p4 = [pts(1,4), pts(2,4)]';
building_center = getCenter(p1, p2, p3, p4);
building_center = [building_center; 1];

gf_points = [pts; ones(1, size(pts,2))];

% %}

% %{ first floor outside

if strcmp(coordinate_system, 'LL')

  % Convert points in to UTM
  for i=1:size(first_floor_outside_points_in,2)
    [first_floor_outside_points_in(1, i), first_floor_outside_points_in(2, i)] = LLToUTM(first_floor_outside_points_in(1, i), first_floor_outside_points_in(2, i));
  end

end

% Calculate points in local coordinates
first_floor_outside_points(1, :) = first_floor_outside_points_in(1, :) - utm_origin_x;
first_floor_outside_points(2, :) = first_floor_outside_points_in(2, :) - utm_origin_y;

% Get center of building
pts = first_floor_outside_points;
p1 = [pts(1,1), pts(2,1)]';
p2 = [pts(1,2), pts(2,2)]';
p3 = [pts(1,3), pts(2,3)]';
p4 = [pts(1,4), pts(2,4)]';
ff_center = getCenter(p1, p2, p3, p4);
ff_center = [ff_center; 1];

ff_points = [pts; ones(1, size(pts,2))];


% %}

% %{ second floor outside

if strcmp(coordinate_system, 'LL')

  % Convert points in to UTM
  for i=1:size(second_floor_outside_points_in,2)
    [second_floor_outside_points_in(1, i), second_floor_outside_points_in(2, i)] = LLToUTM(second_floor_outside_points_in(1, i), second_floor_outside_points_in(2, i));
  end

end

% Calculate points in local coordinates
second_floor_outside_points(1, :) = second_floor_outside_points_in(1, :) - utm_origin_x;
second_floor_outside_points(2, :) = second_floor_outside_points_in(2, :) - utm_origin_y;

% Get center of building
pts = second_floor_outside_points;
p1 = [pts(1,1), pts(2,1)]';
p2 = [pts(1,2), pts(2,2)]';
p3 = [pts(1,3), pts(2,3)]';
p4 = [pts(1,4), pts(2,4)]';
sf_center = getCenter(p1, p2, p3, p4);
sf_center = [sf_center; 1];

sf_points = [pts; ones(1, size(pts,2))];


% %}

% %{ outdoor fires

% wall 1
fires_outdoor = [];
n_f = size(fires_wall1_ratio,2);
for i=1:n_f
  fires_outdoor = [fires_outdoor [ ...
  pts(1,2) + fires_wall1_ratio(i)*(pts(1,3) - pts(1,2)); ...
  pts(2,2) + fires_wall1_ratio(i)*(pts(2,3) - pts(2,2)); ...
  fires_wall1_height(i)]];
end

% wall 2
n_f = size(fires_wall2_ratio,2);
for i=1:n_f
  fires_outdoor = [fires_outdoor [ ...
  pts(1,3) + fires_wall2_ratio(i)*(pts(1,4) - pts(1,3)); ...
  pts(2,3) + fires_wall2_ratio(i)*(pts(2,4) - pts(2,3)); ...
  fires_wall2_height(i)]];
end

% wall 3
n_f = size(fires_wall3_ratio,2);
for i=1:n_f
  fires_outdoor = [fires_outdoor [ ...
  pts(1,4) + fires_wall3_ratio(i)*(pts(1,1) - pts(1,4)); ...
  pts(2,4) + fires_wall3_ratio(i)*(pts(2,1) - pts(2,4)); ...
  fires_wall3_height(i)]];
end

% %}

% windows
windows = [];
n_walls = 4;
n_w = size(windows_ratio,2);
for j=1:n_walls-1
for i=1:n_w
  windows = [windows [ ...
  pts(1,j) + windows_ratio(i)*(pts(1,j+1) - pts(1,j)); ...
  pts(2,j) + windows_ratio(i)*(pts(2,j+1) - pts(2,j)); ...
  windows_height(i)]];
end
end
for i=1:n_w
  windows = [windows [ ...
  pts(1,n_w) + windows_ratio(i)*(pts(1,1) - pts(1,n_w)); ...
  pts(2,n_w) + windows_ratio(i)*(pts(2,1) - pts(2,n_w)); ...
  windows_height(i)]];
end

fire_arena_write

fire_arena_plot
