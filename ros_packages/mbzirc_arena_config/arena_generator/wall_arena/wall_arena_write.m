
dir = 'generated_configs/';
mkdir(dir);

% i = 1;
% fn = sprintf('%sworld_ball_arena_%d.yaml', dir, i);
% while exist(fn)
%   i = i+1;
%   fn = sprintf('%sworld_ball_arena_%d.yaml', dir, i);
% end

fn = sprintf('%sworld_wall_arena.yaml', dir);

file = fopen(fn, 'w');

arena_write

fprintf(file, "\narena_type: 'wall_arena'\n");

fprintf(file, "\nuav_wall_center: [%.2f, %.2f]\n", uav_wall_center(1), uav_wall_center(2));
fprintf(file, "uav_wall_zone: [\n");
fprintf(file, "  %.2f, %.2f,\n", uav_wall1(1), uav_wall1(2));
fprintf(file, "  %.2f, %.2f,\n", uav_wall2(1), uav_wall2(2));
fprintf(file, "  %.2f, %.2f,\n", uav_wall3(1), uav_wall3(2));
fprintf(file, "  %.2f, %.2f]\n", uav_wall4(1), uav_wall4(2));

fprintf(file, "\nuav_brick_center: [%.2f, %.2f]\n", uav_brick_center(1), uav_brick_center(2));
fprintf(file, "uav_brick_zone: [\n");
fprintf(file, "  %.2f, %.2f,\n", uav_brick1(1), uav_brick1(2));
fprintf(file, "  %.2f, %.2f,\n", uav_brick2(1), uav_brick2(2));
fprintf(file, "  %.2f, %.2f,\n", uav_brick3(1), uav_brick3(2));
fprintf(file, "  %.2f, %.2f]\n", uav_brick4(1), uav_brick4(2));

fprintf(file, "\nugv_wall_center: [%.2f, %.2f]\n", ugv_wall_center(1), ugv_wall_center(2));
fprintf(file, "ugv_wall_zone: [\n");
fprintf(file, "  %.2f, %.2f,\n", ugv_wall1(1), ugv_wall1(2));
fprintf(file, "  %.2f, %.2f,\n", ugv_wall2(1), ugv_wall2(2));
fprintf(file, "  %.2f, %.2f,\n", ugv_wall3(1), ugv_wall3(2));
fprintf(file, "  %.2f, %.2f]\n", ugv_wall4(1), ugv_wall4(2));

fprintf(file, "\nugv_brick_center: [%.2f, %.2f]\n", ugv_brick_center(1), ugv_brick_center(2));
fprintf(file, "ugv_brick_zone: [\n");
fprintf(file, "  %.2f, %.2f,\n", ugv_brick1(1), ugv_brick1(2));
fprintf(file, "  %.2f, %.2f,\n", ugv_brick2(1), ugv_brick2(2));
fprintf(file, "  %.2f, %.2f,\n", ugv_brick3(1), ugv_brick3(2));
fprintf(file, "  %.2f, %.2f]\n", ugv_brick4(1), ugv_brick4(2));


fclose(file);

type(fn);
