
includeColors;

% Prepare the figure
hfig = figure(1);
clf
axis equal
hold on
pointBuffer1 = [];
segmentBuffer1 = [];

% Prepare measured points
for i=1:8
  pointBuffer1 = plotPoint(pointBuffer1, points(:, i), BLACK, 10);
  text(points(1,i)+1, points(2,i)+1, sprintf("%d", i));
end

% Prepare line segmets of arena edges
segmentBuffer1 = plotSegment(segmentBuffer1, points(:, 1), points(:, 2), BLACK, 1);
segmentBuffer1 = plotSegment(segmentBuffer1, points(:, 2), points(:, 3), BLACK, 1);
segmentBuffer1 = plotSegment(segmentBuffer1, points(:, 3), points(:, 4), BLACK, 1);
segmentBuffer1 = plotSegment(segmentBuffer1, points(:, 4), points(:, 5), BLACK, 1);
segmentBuffer1 = plotSegment(segmentBuffer1, points(:, 5), points(:, 6), BLACK, 1);
segmentBuffer1 = plotSegment(segmentBuffer1, points(:, 6), points(:, 7), BLACK, 1);
segmentBuffer1 = plotSegment(segmentBuffer1, points(:, 7), points(:, 8), BLACK, 1);
segmentBuffer1 = plotSegment(segmentBuffer1, points(:, 8), points(:, 1), BLACK, 1);

% Prepare arena edge points
pointBuffer1 = plotPoint(pointBuffer1, K, BLUE, 10);
pointBuffer1 = plotPoint(pointBuffer1, L, BLUE, 10);
pointBuffer1 = plotPoint(pointBuffer1, M, BLUE, 10);
pointBuffer1 = plotPoint(pointBuffer1, N, BLUE, 10);

to = 2;
text(K(1)-to, K(2)-to, "K");
text(L(1)-to, L(2)-to, "L");
text(M(1)-to, M(2)-to, "M");
text(N(1)-to, N(2)-to, "N");

% Prepare line segmets of safety area
n_sp = size(safety_area,2);
for i=1:n_sp-1
  segmentBuffer1 = plotSegment(segmentBuffer1, safety_area(:, i), safety_area(:, i+1), RED, 1);
  pointBuffer1 = plotPoint(pointBuffer1, safety_area(:, i), RED, 10);
  text(safety_area(1,i)-1, safety_area(2,i)-1, sprintf("%d", i));
end
segmentBuffer1 = plotSegment(segmentBuffer1, safety_area(:, n_sp), safety_area(:, 1), RED, 1);
pointBuffer1 = plotPoint(pointBuffer1, safety_area(:, n_sp), RED, 10);
text(safety_area(1,n_sp)-1, safety_area(2,n_sp)-1, sprintf("%d", n_sp));

% Prepare center point
pointBuffer1 = plotPoint(pointBuffer1, C, BLACK, 10);

% Prepare corners of take-off area
pointBuffer1 = plotPoint(pointBuffer1, takeoff_center, BLACK, 10);
pointBuffer1 = plotPoint(pointBuffer1, take1, GREEN, 10);
pointBuffer1 = plotPoint(pointBuffer1, take2, GREEN, 10);
pointBuffer1 = plotPoint(pointBuffer1, take3, GREEN, 10);
pointBuffer1 = plotPoint(pointBuffer1, take4, GREEN, 10);

