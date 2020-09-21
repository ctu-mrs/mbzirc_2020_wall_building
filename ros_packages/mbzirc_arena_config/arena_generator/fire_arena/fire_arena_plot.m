arena_plot

% %{ ground floor

% Prepare center point
pointBuffer1 = plotPoint(pointBuffer1, building_center, BLACK, 10);

% Prepare outdoor points
for i=1:4
  pointBuffer1 = plotPoint(pointBuffer1, gf_points(:, i), BLUE, 10);
  % text(gfo_pts_rot(1,i)-1, gfo_pts_rot(2,i)-1, sprintf("%d", i));
end

% %}

% %{ first_floor

% Prepare center point
pointBuffer1 = plotPoint(pointBuffer1, ff_center, BLACK, 10);

% Prepare outdoor points
for i=1:4
  pointBuffer1 = plotPoint(pointBuffer1, ff_points(:, i), CYAN, 10);
  % text(ffo_pts_rot(1,i)-1, ffo_pts_rot(2,i)-1, sprintf("%d", i));
end

% %}

% %{ second_floor

% Prepare center point
pointBuffer1 = plotPoint(pointBuffer1, sf_center, BLACK, 10);

% Prepare outdoor points
for i=1:4
  pointBuffer1 = plotPoint(pointBuffer1, sf_points(:, i), YELLOW, 10);
  % text(sfo_pts_rot(1,i)-1, sfo_pts_rot(2,i)-1, sprintf("%d", i));
end

% %}

% %{ prepare outdoor fires

% Prepare outdoor fires
for i=1:size(fires_outdoor,2)
  pointBuffer1 = plotPoint(pointBuffer1, [fires_outdoor(1:2, i); 1], RED, 10);
  % text(sfo_pts_rot(1,i)-1, sfo_pts_rot(2,i)-1, sprintf("%d", i));
end

% %}

% %{ prepare windows

% Prepare windows
for i=1:size(windows,2)
  pointBuffer1 = plotPoint(pointBuffer1, [windows(1:2, i); 1], CYAN, 10);
  % text(sfo_pts_rot(1,i)-1, sfo_pts_rot(2,i)-1, sprintf("%d", i));
end

% %}

H = eye(3, 3);
plotPoints(hfig, pointBuffer1, H);
plotSegments(hfig, segmentBuffer1, H);
