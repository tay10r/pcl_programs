#include "bvh_model.h"

#include <optional>

#include <bvh/bvh.hpp>
#include <bvh/node_intersectors.hpp>
#include <bvh/primitive_intersectors.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/sphere.hpp>
#include <bvh/sweep_sah_builder.hpp>

namespace {

using Sphere = bvh::Sphere<double>;

using Bvh = bvh::Bvh<double>;

} // namespace

class BvhModelImpl final
{
public:
  BvhModelImpl(std::uint32_t point_count)
    : spheres_(point_count)
  {
  }

  Sphere& operator[](std::uint32_t index) { return spheres_[index]; }

  void commit()
  {
    bvh::SweepSahBuilder builder(bvh_);

    auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(spheres_.data(), spheres_.size());

    auto global_bbox = bvh::compute_bounding_boxes_union(bboxes.get(), spheres_.size());

    builder.build(global_bbox, bboxes.get(), centers.get(), spheres_.size());
  }

  BvhModel::Hit intersect(const bvh::Vector3<double>& origin, const bvh::Vector3<double>& direction) const
  {
    const bvh::Ray ray(origin, direction);

    bvh::ClosestPrimitiveIntersector<bvh::Bvh<double>, Sphere> primitive_intersector(bvh_, spheres_.data());

    bvh::SingleRayTraverser<bvh::Bvh<double>> traverser(bvh_);

    const auto hit = traverser.traverse(ray, primitive_intersector);

    if (!hit)
      return BvhModel::Hit{};

    return BvhModel::Hit{ hit->distance(), static_cast<std::uint32_t>(hit->primitive_index) };
  }

private:
  std::vector<Sphere> spheres_;

  bvh::Bvh<double> bvh_;
};

BvhModel::BvhModel(std::uint32_t point_count)
  : self_(new BvhModelImpl(point_count))
{
}

BvhModel::~BvhModel()
{
  delete self_;
}

void
BvhModel::commit()
{
  self().commit();
}

void
BvhModel::set_point(std::uint32_t index, double x, double y, double z)
{
  self()[index].origin = bvh::Vector3<double>(x, y, z);
}

void
BvhModel::set_radius(std::uint32_t index, double r)
{
  self()[index].radius = r;
}

BvhModel::Hit
BvhModel::intersect(double x, double y, double z, double dx, double dy, double dz) const
{
  const bvh::Vector3<double> origin(x, y, z);

  const bvh::Vector3<double> direction(dx, dy, dz);

  return self_->intersect(origin, direction);
}
