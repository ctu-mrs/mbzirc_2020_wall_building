
% %{ Simulation arena

coordinate_system = 'UTM'; % 'UTM' (default) or 'LL'

% Center of arena
utm_origin_x = 465710.758973;
utm_origin_y = 5249465.43086;
utm_origin_lat = 47.397743;
utm_origin_lon = 8.545594;

% Safety area points
points_in = [
[465690.759158, 5249440.435294] %#1
[465730.762147, 5249440.427927] %#2
[465735.756646, 5249445.432429] %#3
[465735.760889, 5249485.432998] %#4
[465730.762147, 5249490.433259] %#5
[465690.761574, 5249490.429015] %#6
[465685.761314, 5249485.427238] %#7
[465685.755556, 5249445.428189] %#8
]';

% Take-off area (assuming rectangular)
takeoff_center = [0, 0, 1]';
takeoff_length = 5;
takeoff_width = 5;

% %}


