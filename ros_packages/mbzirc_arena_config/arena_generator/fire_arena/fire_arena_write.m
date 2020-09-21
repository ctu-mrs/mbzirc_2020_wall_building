
dir = 'generated_configs/';
mkdir(dir);

% i = 1;
% fn = sprintf('%sworld_ball_arena_%d.yaml', dir, i);
% while exist(fn)
%   i = i+1;
%   fn = sprintf('%sworld_ball_arena_%d.yaml', dir, i);
% end

fn = sprintf('%sworld_fire_arena.yaml', dir);

file = fopen(fn, 'w');

arena_write

fprintf(file, "\narena_type: 'fire_arena'\n");

fprintf(file, "\nground_floor_floor: %.2f\n", ground_floor_floor);
fprintf(file, "ground_floor_ceiling: %.2f\n", ground_floor_ceiling);
fprintf(file, "ground_floor_center: [%.2f, %.2f]\n", building_center(1), building_center(2));
fprintf(file, "ground_floor_outside_points: [\n");
fprintf(file, "  %.2f, %.2f,\n", gf_points(1,1), gf_points(2,1));
fprintf(file, "  %.2f, %.2f,\n", gf_points(1,2), gf_points(2,2));
fprintf(file, "  %.2f, %.2f,\n", gf_points(1,3), gf_points(2,3));
fprintf(file, "  %.2f, %.2f]\n", gf_points(1,4), gf_points(2,4));

fprintf(file, "\nfirst_floor_floor: %.2f\n", first_floor_floor);
fprintf(file, "first_floor_ceiling: %.2f\n", first_floor_ceiling);
fprintf(file, "first_floor_center: [%.2f, %.2f]\n", ff_center(1), ff_center(2));
fprintf(file, "first_floor_outside_points: [\n");
fprintf(file, "  %.2f, %.2f,\n", ff_points(1,1), ff_points(2,1));
fprintf(file, "  %.2f, %.2f,\n", ff_points(1,2), ff_points(2,2));
fprintf(file, "  %.2f, %.2f,\n", ff_points(1,3), ff_points(2,3));
fprintf(file, "  %.2f, %.2f]\n", ff_points(1,4), ff_points(2,4));

fprintf(file, "\nsecond_floor_floor: %.2f\n", second_floor_floor);
fprintf(file, "second_floor_ceiling: %.2f\n", second_floor_ceiling);
fprintf(file, "second_floor_center: [%.2f, %.2f]\n", sf_center(1), ff_center(2));
fprintf(file, "second_floor_outside_points: [\n");
fprintf(file, "  %.2f, %.2f,\n", sf_points(1,1), sf_points(2,1));
fprintf(file, "  %.2f, %.2f,\n", sf_points(1,2), sf_points(2,2));
fprintf(file, "  %.2f, %.2f,\n", sf_points(1,3), sf_points(2,3));
fprintf(file, "  %.2f, %.2f]\n", sf_points(1,4), sf_points(2,4));

fprintf(file, "\nfires_outdoor: [\n");
for i=1:size(fires_outdoor, 2)
  fprintf(file, "  %.2f, %.2f, %.2f", fires_outdoor(1,i), fires_outdoor(2,i), fires_outdoor(3,i));
  if i == size(fires_outdoor, 2)
    fprintf(file, "]\n");
  else
    fprintf(file, ",\n");
  end
end

fprintf(file, "\nwindows: [\n");
for i=1:size(windows, 2)
  fprintf(file, "  %.2f, %.2f, %.2f", windows(1,i), windows(2,i), windows(3,i));
  if i == size(windows, 2)
    fprintf(file, "]\n");
  else
    fprintf(file, ",\n");
  end
end

fclose(file);

type(fn);
