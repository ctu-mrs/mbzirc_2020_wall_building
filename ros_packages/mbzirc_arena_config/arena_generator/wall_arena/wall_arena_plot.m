arena_plot

% Prepare corners of uav wall area
pointBuffer1 = plotPoint(pointBuffer1, uav_wall_center, BLACK, 10);
pointBuffer1 = plotPoint(pointBuffer1, uav_wall1, BLUE, 10);
pointBuffer1 = plotPoint(pointBuffer1, uav_wall2, BLUE, 10);
pointBuffer1 = plotPoint(pointBuffer1, uav_wall3, BLUE, 10);
pointBuffer1 = plotPoint(pointBuffer1, uav_wall4, BLUE, 10);

% Prepare corners of uav brick area
pointBuffer1 = plotPoint(pointBuffer1, uav_brick_center, BLACK, 10);
pointBuffer1 = plotPoint(pointBuffer1, uav_brick1,RED, 10);
pointBuffer1 = plotPoint(pointBuffer1, uav_brick2,RED, 10);
pointBuffer1 = plotPoint(pointBuffer1, uav_brick3,RED, 10);
pointBuffer1 = plotPoint(pointBuffer1, uav_brick4,RED, 10);

% Prepare corners of ugv wall area
pointBuffer1 = plotPoint(pointBuffer1, ugv_wall_center, BLACK, 10);
pointBuffer1 = plotPoint(pointBuffer1, ugv_wall1, BLACK, 10);
pointBuffer1 = plotPoint(pointBuffer1, ugv_wall2, BLACK, 10);
pointBuffer1 = plotPoint(pointBuffer1, ugv_wall3, BLACK, 10);
pointBuffer1 = plotPoint(pointBuffer1, ugv_wall4, BLACK, 10);

% Prepare corners of ugv brick area
pointBuffer1 = plotPoint(pointBuffer1, ugv_brick_center, BLACK, 10);
pointBuffer1 = plotPoint(pointBuffer1, ugv_brick1, BLACK, 10);
pointBuffer1 = plotPoint(pointBuffer1, ugv_brick2, BLACK, 10);
pointBuffer1 = plotPoint(pointBuffer1, ugv_brick3, BLACK, 10);
pointBuffer1 = plotPoint(pointBuffer1, ugv_brick4, BLACK, 10);

H = eye(3, 3);
plotPoints(hfig, pointBuffer1, H);
plotSegments(hfig, segmentBuffer1, H);
