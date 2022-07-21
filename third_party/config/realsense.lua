pointcloud_to_laser = {
  range_min = 0.1;
  range_max = 5.0;
  angle_min = -math.pi;
  angle_max = math.pi;
  height_max = 0.3;
  height_min = -0.3;
  num_ranges = 1000;
  laser_topic = "rs_cloud2scan";
  pointcloud_topic = "/camera/depth/color/points";
}

