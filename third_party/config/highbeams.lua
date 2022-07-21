pointcloud_to_laser = {
  range_min = 0.2;
  range_max = 10.0;
  angle_min = -math.pi;
  angle_max = math.pi;
  height_max = 0.3;
  height_min = -0.3;
  num_ranges = 1000;
  laser_topic = "cloud2scan";
  pointcloud_topic = "velodyne_points";
}

