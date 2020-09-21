
% GPS coordinates of arena points
% [240054.017162, 2702845.742010]
% [240097.850291, 2702831.853810]
% [240104.127711, 2702833.341839]
% [240111.853206, 2702858.442528]
% [240109.255328, 2702865.063667]
% [240065.868877, 2702877.633356]
% [240059.853755, 2702875.281325]
% [240051.588012, 2702848.902836]
%
% GPS coorinates of safety area
% [24.417407, 54.436501]
% [24.417290, 54.436936]
% [24.417316, 54.436946]
% [24.417303, 54.436996]
% [24.417531, 54.437067]
% [24.417547, 54.437027]
% [24.417593, 54.437044]
% [24.417702, 54.436606]
% [24.417665, 54.436593]
% [24.417677, 54.436553]
% [24.417437, 54.436474]
% [24.417430, 54.436504]

% %{ ADNEC fire after rehearsal 1

arena_id = 'fire_adnec_after_rehearsal_1';

coordinate_system = 'LL'; % 'UTM' (default) or 'LL'

% Center of arena
utm_origin_x = 0.0;
utm_origin_y = 0.0;
utm_origin_lat = 24.41757;
utm_origin_lon = 54.43665;

% Points to calculate arena corners (8 points must be supplied)
points_in = [
[24.417407, 54.436501], %#1
[24.417290, 54.436936], %#2
[24.417303, 54.436996], %#4
[24.417531, 54.437067], %#5
[24.417593, 54.437044], %#7
[24.417702, 54.436606], %#8
[24.417677, 54.436553], %#10
[24.417437, 54.436474], %#11
]';

safety_area_in = [
[24.417407, 54.436501], %#1
[24.417290, 54.436936], %#2
[24.417316, 54.436946], %#3
[24.417303, 54.436996], %#4
[24.417531, 54.437067], %#5
[24.417547, 54.437027], %#6
[24.417593, 54.437044], %#7
[24.417702, 54.436606], %#8
[24.417665, 54.436593], %#9
[24.417677, 54.436553], %#10
[24.417437, 54.436474], %#11
[24.417430, 54.436504] %#12
]';

min_height = 0.5;
max_height = 18.0;

% Take-off area (assuming rectangular)
takeoff_center = [8, -22, 1]';
takeoff_length = 20;
takeoff_width = 5;

% Building definition

% Position of the tower corner closer to K
building_pos = [465715.758973, 5249475.43086];

% %{ ground floor

ground_floor_floor = 0.0;
ground_floor_ceiling = 4.8;
tower_offset = [5.0, 0.0]';

ground_floor_outside_points_in = [
[24.417426, 54.436749]
[24.417366, 54.436991]
[24.417488, 54.437027]
[24.417547, 54.436788]
]';

% %}

% %{ first floor

first_floor_floor = 5.0;
first_floor_ceiling = 9.8;

% latlon
first_floor_outside_points_in = [
[24.417426, 54.436749]
[24.417396, 54.436873]
[24.417518, 54.436914]
[24.417547, 54.436788]
]';

% %}

% %{ second floor

second_floor_floor = 10.0;
second_floor_ceiling = 14.8;

second_floor_outside_points_in = [
[24.417426, 54.436749]
[24.417396, 54.436873]
[24.417518, 54.436914]
[24.417547, 54.436788]
]';

% %}

% %{ outdoor fires

% wall 0 - south in default configuration (KL edge)
% no fires on this wall

% wall 1 - east in default configuration (LM edge)
fires_wall1_ratio = [0.072, 0.088, 0.48, 0.96, 0.88];
fires_wall1_height = [13.2, 8.5, 11, 13.2, 5.9];

% wall 2 - north in default configuration (MN edge)
fires_wall2_ratio = [0.0543, 0.1451, 0.5346, 0.9147, 0.9211];
fires_wall2_height = [1.0, 13.26, 10.9, 13.3, 6];

% wall 3 - west in default configuration (NK edge)
fires_wall3_ratio = [0.072, 0.0934, 0.376, 0.8545, 0.876];
fires_wall3_height = [3.3, 10.65, 11, 13.6, 6.38];

% %}

% %{ windows TODO

% windows ahve the same ratios and height for all sides
windows_ratio = [0.3475, 0.3475, 0.6545, 0.6545];
windows_height = [12.6, 7.265, 12.6, 7.265];

% %}

% %}

% % %{ ADNEC whole arena

% arena_id = 'fire_adnec_whole';

% coordinate_system = 'LL'; % 'UTM' (default) or 'LL'

% % Center of arena
% utm_origin_x = 0.0;
% utm_origin_y = 0.0;
% utm_origin_lat = 24.41747;
% utm_origin_lon = 54.43646;

% % Ball arena (8 points must be supplied)
% points_in = [
%           [24.41753, 54.43519], %#1
%           [24.41684, 54.43772], %#2
%           [24.41688, 54.4378], %#3
%           [24.41726, 54.43786], %#4
%           [24.41734, 54.43782], %#5
%           [24.41802, 54.43537], %#6
%           [24.41798, 54.43532], %#7
%           [24.41759, 54.43516] %#8
%           ]';

% % Take-off area (assuming rectangular)
% takeoff_center = [0, 0, 1]';
% takeoff_length = 5;
% takeoff_width = 5;

% % Building definition

% % Position of the tower corner closer to K
% building_pos = [465715.758973, 5249475.43086];

% % %{ ground floor

% ground_floor_floor = 0.0;
% ground_floor_ceiling = 4.8;
% tower_offset = [5.0, 0.0]';

% ground_floor_outside_points = [
%                      [0.0, 0.0]
%                      [20.0, 0.0]
%                      [20.0, 10.0]
%                      [0.0, 10.0]
%                      ]';

% % %}

% % %{ first floor

% first_floor_floor = 5.0;
% first_floor_ceiling = 9.8;

% first_floor_outside_points = [
%                      [0.0, 0.0]
%                      [10.0, 0.0]
%                      [10.0, 10.0]
%                      [0.0, 10.0]
%                      ]';

% % %}

% % %{ second floor

% second_floor_floor = 10.0;
% second_floor_ceiling = 14.8;

% second_floor_outside_points = [
%                      [0.0, 0.0]
%                      [10.0, 0.0]
%                      [10.0, 10.0]
%                      [0.0, 10.0]
%                      ]';

% % %}

% % %}

%% %{ simulation

%coordinate_system = 'UTM'; % 'UTM' (default) or 'LL'

%%for simulation of ch. 2
%utm_origin_x = 465710.758973
%utm_origin_y = 5249465.43086

%%simulation of ch. 2 (new size)
%points_in = [
%            [465731.023182, 5249489.377807] %#4
%            [465688.667804, 5249484.083387] %#5
%            [465686.924522, 5249479.240928] %#6
%            [465686.924522, 5249452.123161] %#7
%            [465690.992185, 5249446.828737] %#8
%            [465730.442081, 5249450.250742] %#1
%            [465733.993221, 5249455.416032] %#2
%            [465734.509750, 5249486.601465] %#3
%            ]';

%% Take-off area (assuming rectangular)
%takeoff_center = [-20, -10, 1]';
%takeoff_length = 3;
%takeoff_width = 2;

%% Building definition

%% Position of the tower corner closer to K
%building_pos = [465715.758973, 5249475.43086];

%% %{ ground floor

%ground_floor_floor = 0.0;
%ground_floor_ceiling = 4.8;
%tower_offset = [5.0, 0.0]';

%ground_floor_outside_points = [
%                     [0.0, 0.0]
%                     [20.0, 0.0]
%                     [20.0, 10.0]
%                     [0.0, 10.0]
%                     ]';

%% %}

%% %{ first floor

%first_floor_floor = 5.0;
%first_floor_ceiling = 9.8;

%first_floor_outside_points = [
%                     [0.0, 0.0]
%                     [10.0, 0.0]
%                     [10.0, 10.0]
%                     [0.0, 10.0]
%                     ]';

%% %}

%% %{ second floor

%second_floor_floor = 10.0;
%second_floor_ceiling = 14.8;

%second_floor_outside_points = [
%                     [0.0, 0.0]
%                     [10.0, 0.0]
%                     [10.0, 10.0]
%                     [0.0, 10.0]
%                     ]';

%% %}

%% %}
