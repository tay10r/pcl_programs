#include "common/arg_parser.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <random>

int
main(int argc, char** argv)
{
  ArgParser arg_parser(argc, argv);

  const auto seed = arg_parser.find_uint32("--seed", 1234);

  const auto N = arg_parser.find_uint32("--count", 1'000);

  const auto path = arg_parser.find_string("--output", "box.pcd");

  pcl::PointCloud<pcl::PointXYZ> points;

  points.resize(N);

  std::mt19937 rng(seed);

  std::uniform_real_distribution<float> dist(-1, 1);

  for (std::uint32_t i = 0; i < N; i++) {
    points[i].x = dist(rng);
    points[i].y = dist(rng);
    points[i].z = dist(rng);
  }

  pcl::PCDWriter writer;

  writer.writeBinaryCompressed(path, points);

  return EXIT_SUCCESS;
}
