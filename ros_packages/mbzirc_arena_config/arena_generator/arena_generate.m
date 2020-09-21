
% Convert to UTM when arena specified in Latitude and Longitude
if strcmp(coordinate_system, 'LL')

  % Convert origin to UTM
  [utm_origin_x, utm_origin_y] = LLToUTM(utm_origin_lat, utm_origin_lon);

  % Convert points in to UTM
  for i=1:size(points_in,2)
    [points_in(1, i), points_in(2, i)] = LLToUTM(points_in(1, i), points_in(2, i));
  end

  % Convert safety area to UTM
  for i=1:size(safety_area_in,2)
    [safety_area_in(1, i), safety_area_in(2, i)] = LLToUTM(safety_area_in(1, i), safety_area_in(2, i));
  end

end

% Calculate points in local coordinates
points(1, :) = points_in(1, :) - utm_origin_x;
points(2, :) = points_in(2, :) - utm_origin_y;

% Calculate safety area in local coordinates
safety_area(1, :) = safety_area_in(1, :) - utm_origin_x;
safety_area(2, :) = safety_area_in(2, :) - utm_origin_y;

% Homogeneous coordinates
points = [points; ones(1, size(points,2))];
safety_area = [safety_area; ones(1, size(safety_area,2))];

% Arena edge lines from points
line1 = cross(points(:, 1), points(:, 2));
line2 = cross(points(:, 3), points(:, 4));
line3 = cross(points(:, 5), points(:, 6));
line4 = cross(points(:, 7), points(:, 8));

% Determine arena edge points by intersecting the edge lines
K = linesIntersection(line1, line4);
L = linesIntersection(line1, line2);
M = linesIntersection(line2, line3);
N = linesIntersection(line3, line4);

% Get arena center
dline1 = cross(K, M);
dline2 = cross(L, N);

C = linesIntersection(dline1, dline2);

% Calculate the angle between first arena edge and east axis of UTM origin (ENU)
ang_diff = linesAngle(line1, [0, 1, 0]');

% Sanitize angle
if ang_diff < pi/2
  ang_diff = ang_diff + pi;
end
if ang_diff > pi/2
  ang_diff = ang_diff - pi;
end

if line1(2) < 0
  ang_diff = -ang_diff;
end

% Calculate corners of take-off area
[take1, take2, take3, take4] = getRect(takeoff_center, ang_diff, takeoff_length, takeoff_width);
