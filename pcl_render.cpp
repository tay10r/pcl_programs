#include "common/arg_parser.h"
#include "common/bvh_model.h"
#include "common/stb_image_write.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <iostream>
#include <random>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

namespace {

using Vec3f = Eigen::Matrix<double, 3, 1>;

unsigned char
to_color_byte(double x)
{
  const float color = x * 255;
  if (color < 0.0)
    return 0;
  else if (color > 255.0)
    return 255;
  else
    return color;
}

template<typename Rng>
Vec3f
sample_hemisphere(const Vec3f& normal, Rng& rng)
{
  std::uniform_real_distribution<double> dist(-1, 1);

  for (int i = 0; i < 32; i++) {
    const Vec3f v(dist(rng), dist(rng), dist(rng));
    if ((v.dot(v) <= 1.0) && (v.dot(normal) >= 0.0))
      return v.normalized();
  }

  return normal;
}

template<typename Rng>
Vec3f
trace(const pcl::PointCloud<pcl::PointXYZ>& points,
      const pcl::PointCloud<pcl::PointXYZ>& albedo,
      const pcl::PointCloud<pcl::PointXYZ>& emission,
      const BvhModel& bvh_model,
      const Vec3f& origin,
      const Vec3f& direction,
      Rng& rng,
      int depth)
{
  const int max_depth = 3;

  if (depth > max_depth)
    return Vec3f(0, 0, 0);

  const auto hit = bvh_model.intersect(origin[0], origin[1], origin[2], direction[0], direction[1], direction[2]);
  if (!hit)
    // return Vec3f(1, 1, 1) * ((direction[1] + 1.0) * 0.5);
    return Vec3f(0, 0, 0);

  const auto& point = points[hit.index];

  const Vec3f point_center(point.x, point.y, point.z);

  const Vec3f hit_point = origin + (direction * hit.distance);

  const Vec3f hit_normal = (hit_point - point_center).normalized();

  const auto reflection_direction = sample_hemisphere(hit_normal, rng);

  const double shadow_bias = 1.0e-6;

  const auto reflection_origin = origin + (direction * (hit.distance - shadow_bias));

  const auto indirect_light =
    trace(points, albedo, emission, bvh_model, reflection_origin, reflection_direction, rng, depth + 1);

  const Vec3f point_albedo(double(albedo[hit.index].x), double(albedo[hit.index].y), double(albedo[hit.index].z));

  const Vec3f point_emission(
    double(emission[hit.index].x), double(emission[hit.index].y), double(emission[hit.index].z));

  return point_albedo.cwiseProduct(indirect_light) + point_emission;
}

} // namespace

int
main(int argc, char** argv)
{
  ArgParser arg_parser(argc, argv);

  const auto w = arg_parser.find_uint32("--width", 640);

  const auto h = arg_parser.find_uint32("--height", 480);

  const auto spp = arg_parser.find_uint32("--spp", 16);

  const auto fov = arg_parser.find_float("--fov", 45.0f);

  const auto radius = arg_parser.find_float("--radius", 1.0e-3);

  const auto points_path = arg_parser.find_string("--points", "points.pcd");

  const auto albedo_path = arg_parser.find_string("--albedo", "");

  const auto emission_path = arg_parser.find_string("--emission", "");

  const auto output_path = arg_parser.find_string("--output", "output.png");

  pcl::PointCloud<pcl::PointXYZ> points;

  pcl::PCDReader reader;

  if (reader.read(points_path, points) != 0) {
    std::cerr << "Failed to read " << points_path << std::endl;
    return EXIT_FAILURE;
  }

  pcl::PointCloud<pcl::PointXYZ> albedo;

  if (albedo_path.empty()) {
    albedo.resize(points.size());
    for (std::size_t i = 0; i < albedo.size(); i++) {
      albedo[i].x = 0.8;
      albedo[i].y = 0.8;
      albedo[i].z = 0.8;
    }
  } else {
    if (reader.read(albedo_path, albedo) != 0) {
      std::cerr << "Failed to read " << albedo_path << std::endl;
      return EXIT_FAILURE;
    }
  }

  pcl::PointCloud<pcl::PointXYZ> emission;

  if (emission_path.empty()) {
    emission.resize(points.size());
    for (std::size_t i = 0; i < emission.size(); i++) {
      emission[i].x = 0.0;
      emission[i].y = 0.0;
      emission[i].z = 0.0;
    }
  } else {
    if (reader.read(emission_path, emission) != 0) {
      std::cerr << "Failed to read " << emission_path << std::endl;
      return EXIT_FAILURE;
    }
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

      color += trace(points, albedo, emission, bvh_model, origin, direction, rng, 0);
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
