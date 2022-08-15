#include "arg_parser.h"
#include "bvh_model.h"
#include "stb_image_write.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <iostream>

int
main(int argc, char** argv)
{
  ArgParser arg_parser(argc, argv);

  const auto position_path = arg_parser.find_string("--position", "position.pcd");

  const auto mass_path = arg_parser.find_string("--mass", "mass.pcd");

  const auto charge_path = arg_parser.find_string("--charge", "charge.pcd");

  const auto output_path = arg_parser.find_string("--output", "output.pcd");

  pcl::PointCloud<pcl::PointXYZ> points;

  pcl::PCDReader reader;

  if (reader.read(path, points) != 0) {
    std::cerr << "Failed to read " << path << std::endl;
    return EXIT_FAILURE;
  }

  BvhModel bvh_model(points.size());

  for (std::size_t i = 0; i < points.size(); i++) {

    bvh_model.set_point(i, points[i].x, points[i].y, points[i].z);

    bvh_model.set_radius(i, radius);
  }

  bvh_model.commit();

  std::vector<unsigned char> tmp(w * h * 3);

  const double u_scale = 1.0 / double(w);
  const double v_scale = 1.0 / double(h);

  const double aspect = double(w) / h;

  const double fov_radians = fov * double(M_PI) / 180.0;

  const double fovy = std::tan(fov_radians * 0.5);

#pragma omp parallel for

  for (std::size_t i = 0; i < (w * h); i++) {

    const auto x = i % w;
    const auto y = i / w;

    std::uniform_real_distribution<double> dist(0, 1);

    std::minstd_rand rng(i);

    Vec3f color(0, 0, 0);

    for (std::size_t j = 0; j < spp; j++) {

      const auto u = (double(x) + dist(rng)) * u_scale;
      const auto v = (double(y) + dist(rng)) * v_scale;

      const double dx = ((u * 2) - 1) * fovy * aspect;
      const double dy = (1 - (v * 2)) * fovy;
      const double dz = -1;

      Vec3f origin(0, 0, 0);

      Vec3f direction = Vec3f(dx, dy, dz).normalized();

      color += trace(points, bvh_model, origin, direction, rng, 0);
    }

    const unsigned char r = to_color_byte(color[0] / double(spp));
    const unsigned char g = to_color_byte(color[1] / double(spp));
    const unsigned char b = to_color_byte(color[2] / double(spp));

    tmp[(i * 3) + 0] = r;
    tmp[(i * 3) + 1] = g;
    tmp[(i * 3) + 2] = b;
  }

  stbi_write_png(output_path.c_str(), w, h, 3, &tmp[0], w * 3);

  return EXIT_SUCCESS;
}
