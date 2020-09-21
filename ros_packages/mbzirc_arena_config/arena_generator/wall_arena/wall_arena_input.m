
% %{ ADNEC wall arena after rehearsal 1

arena_id = 'wall_adnec_after_rehearsal1';

coordinate_system = 'LL'; % 'UTM' (default) or 'LL'

% Center of arena
utm_origin_x = 0.0;
utm_origin_y = 0.0;
utm_origin_lat = 24.4173038;
utm_origin_lon = 54.4373136;

% Ball arena (8 points must be supplied)
points_in = [
          [24.4172242, 54.4370922], %#1
          [24.4171363, 54.4374476], %#2
          [24.4171463, 54.4374837], %#3
          [24.4173254, 54.4375675], %#4
          [24.4174054, 54.4375033], %#5
          [24.4174912, 54.4372169], %#6
          [24.4174548, 54.4371163], %#7
          [24.4172723, 54.437058] %#8
          ]';

safety_area_in = [
          [24.4172242, 54.4370922], %#1
          [24.4171363, 54.4374476], %#2
          [24.4171463, 54.4374837], %#3
          [24.4173254, 54.4375675], %#4
          [24.4174054, 54.4375033], %#5
          [24.4174912, 54.4372169], %#6
          [24.4174548, 54.4371163], %#7
          [24.4172723, 54.437058] %#8
          ]';

min_height = 0.5;
max_height = 18.0;

% Take-off area (assuming rectangular)
takeoff_center = [-5, -10, 1]';
takeoff_length = 20;
takeoff_width = 5;

% UAV wall area (assuming rectangular)
uav_wall_center = [15, 5, 1]';
uav_wall_length = 8;
uav_wall_width = 4;

% UAV brick area (assuming rectangular)
uav_brick_center = [-10, 5, 1]';
uav_brick_length = 5;
uav_brick_width = 8;

% UGV wall area (assuming rectangular)
ugv_wall_center = [10, -5, 1]';
ugv_wall_length = 8;
ugv_wall_width = 4;

% UGV brick area (assuming rectangular)
ugv_brick_center = [-15, 0, 1]';
ugv_brick_length = 8;
ugv_brick_width = 3;

% %}

% % %{ ADNEC whole arena

% arena_id = 'wall_adnec_whole';

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

% % UAV wall area (assuming rectangular)
% uav_wall_center = [-17, 2, 1]';
% uav_wall_length = 5;
% uav_wall_width = 1;

% % UAV brick area (assuming rectangular)
% uav_brick_center = [-10, -12, 1]';
% uav_brick_length = 5;
% uav_brick_width = 1;

% % UGV wall area (assuming rectangular)
% ugv_wall_center = [7, 7, 1]';
% ugv_wall_length = 5;
% ugv_wall_width = 1;

% % UGV brick area (assuming rectangular)
% ugv_brick_center = [10, -5, 1]';
% ugv_brick_length = 5;
% ugv_brick_width = 1;

% % %}

%% %{ simulation

%% Center of arena (measure or calculate?)
%% utm_origin_x = 257871.572223;  
%% utm_origin_y = 2709427.58274; 
%% utm_origin_x = 330604.247983
%% utm_origin_y = 2669875.693440

%%for simulation of ch. 2
%utm_origin_x = 465710.758973
%utm_origin_y = 5249465.43086


%% Ball arena (8 points must be supplied)
%% points_in = [
%%           [257901.516843, 2709459.03535], %#1
%%           [257834.627666, 2709452.37063], %#2
%%           [257828.092666, 2709445.33886], %#3
%%           [257832.537506, 2709401.34353], %#4
%%           [257837.622555, 2709395.70418], %#5
%%           [257904.515696, 2709401.89817], %#6
%%           [257916.559927, 2709411.3335], %#7
%%           [257912.711861, 2709453.85759] %#8
%%           ]';

%%simulation of ch. 2
%%points_in = [
%%            [465694.534202, 5249455.804396] %#2
%%            [465721.468139, 5249458.771970] %#3
%%            [465725.231522, 5249462.145234] %#4
%%            [465723.326606, 5249472.148300] %#5
%%            [465714.586869, 5249475.015026] %#6
%%            [465695.414551, 5249471.012260] %#7
%%            [465690.849509, 5249465.819979] %#8
%%            [465692.581747, 5249457.947637] %#1
%%            ]';

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

%%points_in = [
%%          [330588.947982, 2669857.293436]
%%          [330623.947984, 2669867.993438]
%%          [330625.447982, 2669873.093439]
%%          [330613.247980, 2669885.993440]
%%          [330608.547985, 2669888.193442]
%%          [330581.747981, 2669881.093441]
%%          [330579.347978, 2669875.993439]
%%          [330584.247983, 2669860.693440]
%%          ]';

%% Take-off area (assuming rectangular)
%takeoff_center = [0, 0, 1]';
%takeoff_length = 3;
%takeoff_width = 2;

%% UAV wall area (assuming rectangular)
%uav_wall_center = [-17, 2, 1]';
%uav_wall_length = 5;
%uav_wall_width = 1;

%% UAV brick area (assuming rectangular)
%uav_brick_center = [-10, -12, 1]';
%uav_brick_length = 5;
%uav_brick_width = 1;

%% UGV wall area (assuming rectangular)
%ugv_wall_center = [7, 7, 1]';
%ugv_wall_length = 5;
%ugv_wall_width = 1;

%% UGV brick area (assuming rectangular)
%ugv_brick_center = [10, -5, 1]';
%ugv_brick_length = 5;
%ugv_brick_width = 1;

%% %}
