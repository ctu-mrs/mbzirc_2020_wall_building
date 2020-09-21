
% %{ ADNEC ball arena after rehearsal 1

arena_id = 'ball_adnec_after_reahearsal1';

coordinate_system = 'LL'; % 'UTM' (default) or 'LL'

% Center of arena
utm_origin_x = 0.0;
utm_origin_y = 0.0;
utm_origin_lat = 24.417678;
utm_origin_lon = 54.435881;

% Safety area points (can be non-convex)
safety_area_in = [
          [24.4176491, 54.4355062], %#1
          [24.4175669, 54.4358474], %#2
          [24.4175666, 54.435868], %#3
          [24.4175316, 54.4359729], %#4
          [24.4174338, 54.4363507], %#5
          [24.4174423, 54.4363678], %#6
          [24.417667, 54.4364356], %#7
          [24.4176781, 54.4364042], %#8
          [24.4177245, 54.4364181], %#9
          [24.4178259, 54.4360718], %#10
          [24.4177789, 54.4360469], %#11
          [24.417803, 54.4359456], %#12
          [24.4178442, 54.4359509], %#13
          [24.4179288, 54.4356065], %#14
          [24.4178968, 54.4355905], %#15
          [24.4179032, 54.4355481], %#16
          [24.4176883, 54.4354886], %#17
          [24.4176718, 54.4355055] %#18
          ]';

% Points used for calculation of corners (8)
points_in = [
          [24.4176491, 54.4355062], %#1
          [24.4174338, 54.4363507], %#2
          [24.4174423, 54.4363678], %#3
          [24.417667, 54.4364356], %#4
          [24.4177245, 54.4364181], %#5
          [24.4179288, 54.4356065], %#6
          [24.4179032, 54.4355481], %#7
          [24.4176883, 54.4354886] %#8
          ]';

min_height = 0.5;
max_height = 18.0;

% Take-off area (assuming rectangular)
takeoff_center = [0, 0, 1]';
takeoff_length = 5;
takeoff_width = 5;

% Drop-off area (assuming rectangular)
dropoff_in = [
[24.4175746, 54.4358814],
[24.4175464, 54.4359734],
[24.4176428, 54.4360068],
[24.4176654, 54.4359116]
]';

% %}

%% %{ ADNEC whole arena

%arena_id = 'ball_adnec_whole';

%coordinate_system = 'LL'; % 'UTM' (default) or 'LL'

%% Center of arena
%utm_origin_x = 0.0;
%utm_origin_y = 0.0;
%utm_origin_lat = 24.41747;
%utm_origin_lon = 54.43646;

%%simulation of ch. 2 (new size)
%points_in = [
%          [24.41753, 54.43519], %#1
%          [24.41684, 54.43772], %#2
%          [24.41688, 54.4378], %#3
%          [24.41726, 54.43786], %#4
%          [24.41734, 54.43782], %#5
%          [24.41802, 54.43537], %#6
%          [24.41798, 54.43532], %#7
%          [24.41759, 54.43516] %#8
%          ]';

%% Take-off area (assuming rectangular)
%takeoff_center = [0, 0, 1]';
%takeoff_length = 5;
%takeoff_width = 5;

%% Drop-off area (assuming rectangular)
%dropoff_center = [10, 0, 1]';
%dropoff_length = 5;
%dropoff_width = 5;

%% %}

% % %{ Ball desert shit arena

% coordinate_system = 'LL'; % 'UTM' (default) or 'LL'

% % Center of arena
% utm_origin_x = 0.0;
% utm_origin_y = 0.0;
% utm_origin_lat = 24.13226;
% utm_origin_lon = 55.33299;

% % Ball arena (8 points must be supplied)
% points_in = [
%           [24.13234, 55.33271], %#1
%           [24.13241, 55.33309], %#2
%           [24.13235, 55.33315], %#3
%           [24.1322, 55.33319], %#4
%           [24.13213, 55.33316], %#5
%           [24.13206, 55.33283], %#6
%           [24.1321, 55.33278], %#7
%           [24.13227, 55.33269] %#8
%           ]';

% % Take-off area (assuming rectangular)
% takeoff_center = [-10, 10, 1]';
% takeoff_length = 30;
% takeoff_width = 5;

% % Drop-off area (assuming rectangular)
% dropoff_center = [-15, -10, 1]';
% dropoff_length = 5;
% dropoff_width = 5;

% % %}

