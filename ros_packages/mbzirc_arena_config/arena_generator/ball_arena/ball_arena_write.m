
dir = 'generated_configs/';
mkdir(dir);

% i = 1;
% fn = sprintf('%sworld_ball_arena_%d.yaml', dir, i);
% while exist(fn)
%   i = i+1;
%   fn = sprintf('%sworld_ball_arena_%d.yaml', dir, i);
% end

fn = sprintf('%sworld_ball_arena.yaml', dir);

file = fopen(fn, 'w');

arena_write

fprintf(file, "\narena_type: 'ball_arena'\n");

fprintf(file, "\ndropoff_center: [%.2f, %.2f]\n", dropoff_center(1), dropoff_center(2));
fprintf(file, "dropoff_zone: [\n");
for i=1:size(dropoff, 2)
  fprintf(file, "    %.2f, %.2f", dropoff(1, i), dropoff(2, i));
  if i == size(dropoff, 2)
    fprintf(file, "]\n");
  else
    fprintf(file, ",\n");
  end
end

fclose(file);

type(fn);
