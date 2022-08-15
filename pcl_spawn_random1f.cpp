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

  const auto seed = arg_parser.find_uint32("--seed", 1234);

  const auto min_boundary = arg_parser.find_float("--min", 0.0f);
  const auto max_boundary = arg_parser.find_float("--max", 1.0f);

  const auto path = arg_parser.find_string("--output", "output.pcd");

  pcl::PointCloud<pcl::Intensity> points;

  points.resize(N);

  std::mt19937 rng(seed);

  std::uniform_real_distribution<float> dist(min_boundary, max_boundary);

  for (std::uint32_t i = 0; i < N; i++) {
    points[i].intensity = dist(rng);
  }

  pcl::PCDWriter writer;

  writer.writeBinaryCompressed(path, points);

  return EXIT_SUCCESS;
}
