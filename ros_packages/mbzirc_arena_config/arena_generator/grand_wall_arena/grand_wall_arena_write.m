
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

fclose(file);

type(fn);
