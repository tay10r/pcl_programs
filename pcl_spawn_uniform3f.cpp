#include "common/arg_parser.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <random>

int
main(int argc, char** argv)
{
  ArgParser arg_parser(argc, argv);

  const auto N = arg_parser.find_uint32("--count", 1'000);

  const auto x = arg_parser.find_float("--x", 0.0f);
  const auto y = arg_parser.find_float("--y", 0.0f);
  const auto z = arg_parser.find_float("--z", 0.0f);

  const auto path = arg_parser.find_string("--output", "output.pcd");

  pcl::PointCloud<pcl::PointXYZ> points;

  points.resize(N);

  for (std::uint32_t i = 0; i < N; i++) {
    points[i].x = x;
    points[i].y = y;
    points[i].z = z;
  }

  pcl::PCDWriter writer;

  writer.writeBinaryCompressed(path, points);

  return EXIT_SUCCESS;
}
