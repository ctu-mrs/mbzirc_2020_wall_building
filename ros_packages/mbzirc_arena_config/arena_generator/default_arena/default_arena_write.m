
dir = 'generated_configs/';
mkdir(dir);

fn = sprintf('%sworld_default_arena.yaml', dir);

file = fopen(fn, 'w');

arena_write

fprintf(file, "\narena_type: 'default_arena'\n");

fclose(file);

type(fn);
