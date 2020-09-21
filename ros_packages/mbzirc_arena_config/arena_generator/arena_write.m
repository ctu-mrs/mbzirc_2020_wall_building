fprintf(file, "#\n");
fprintf(file, "## DEFINITION OF THE MBZIRC CHALLENGE ARENA\n");
fprintf(file, "#\n");
fprintf(file, " \n");
fprintf(file, "arena_id: \"%s\"\n", arena_id);
fprintf(file, " \n");
fprintf(file, "use_utm_origin: true\n");
fprintf(file, "utm_origin_units: 0 # 0 - UTM, 1 - Latitude Longitude\n");
fprintf(file, "utm_origin_lat: 0\n");
fprintf(file, "utm_origin_lon: 0\n");
fprintf(file, "utm_origin_x: %.2f\n", utm_origin_x);
fprintf(file, "utm_origin_y: %.2f\n", utm_origin_y);
fprintf(file, "\n");
fprintf(file, "use_local_origin: false\n");
fprintf(file, "local_origin_x: 0.0\n");
fprintf(file, "local_origin_y: 0.0\n");
fprintf(file, "\n");
fprintf(file, "safety_area:\n");
fprintf(file, "\n");
fprintf(file, "  use_safety_area: true\n");
fprintf(file, "\n");
fprintf(file, "  frame_name: \"gps_origin\"\n");
fprintf(file, "\n");
fprintf(file, "  polygon_obstacles:\n");
fprintf(file, "    # loaded as a vector of matrices\n");
fprintf(file, "    # each matrix has polygon vertices in columns\n");
fprintf(file, "    # [[M1], [M2]]\n");
fprintf(file, "    enabled: false\n");
fprintf(file, "    data: [1, 10, 10, 1,    5, 8, 5,\n");
fprintf(file, "           1, 1, 10, 10,    5, 5, 8,]\n");
fprintf(file, "    rows: 2 # each matrix has two rows\n");
fprintf(file, "    cols: [4, 3] # nums of cols of each matrix\n");
fprintf(file, "\n");
fprintf(file, "  point_obstacles:\n");
fprintf(file, "    # loaded as a vector of matrices\n");
fprintf(file, "    # x, y, radius\n");
fprintf(file, "    enabled: false\n");
fprintf(file, "    data: [-5.0, -5.0, 2, # 1st point\n");
fprintf(file, "           -10.0, -10.0, 4] # 2nd point\n");
fprintf(file, "    rows: 1 # each matrix has a single row\n");
fprintf(file, "    cols: [3, 3] # nums of cols of each matrix\n");
fprintf(file, "\n");
fprintf(file, "  # convex polygon CCW\n");
fprintf(file, "  # race track\n");
fprintf(file, "  safety_area: [\n");
for i=1:size(safety_area, 2)
  fprintf(file, "    %.2f, %.2f", safety_area(1, i), safety_area(2, i));
  if i == size(safety_area, 2)
    fprintf(file, "]\n");
  else
    fprintf(file, ",\n");
  end
end
fprintf(file, "\n");
fprintf(file, "  max_height: %.2f\n", max_height);
fprintf(file, "  min_height: %.2f\n", min_height);

fprintf(file, "\n\narena_ang_diff: %.6f\n", -ang_diff);
fprintf(file, "arena_center: [%.2f, %.2f]\n", C(1), C(2));
fprintf(file, "arena_corners: [\n");
fprintf(file, "  %.2f, %.2f, # K\n", K(1), K(2));
fprintf(file, "  %.2f, %.2f, # L\n", L(1), L(2));
fprintf(file, "  %.2f, %.2f, # M\n", M(1), M(2));
fprintf(file, "  %.2f, %.2f] # N\n", N(1), N(2));

fprintf(file, "\ntakeoff_center: [%.2f, %.2f]\n", takeoff_center(1), takeoff_center(2));
fprintf(file, "takeoff_zone: [\n");
fprintf(file, "  %.2f, %.2f,\n", take1(1), take1(2));
fprintf(file, "  %.2f, %.2f,\n", take2(1), take2(2));
fprintf(file, "  %.2f, %.2f,\n", take3(1), take3(2));
fprintf(file, "  %.2f, %.2f]\n", take4(1), take4(2));
