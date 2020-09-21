arena_plot

% Prepare center of drop-off area
pointBuffer1 = plotPoint(pointBuffer1, dropoff_center, BLACK, 10);

% Prepare corners of drop-off area
for i=1:size(dropoff, 2)
  pointBuffer1 = plotPoint(pointBuffer1, dropoff(:,i), BLUE, 10);
end


H = eye(3, 3);
plotPoints(hfig, pointBuffer1, H);
plotSegments(hfig, segmentBuffer1, H);
