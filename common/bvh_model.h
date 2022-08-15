#pragma once

#include <limits>

#include <cstdint>

class BvhModelImpl;

class BvhModel final
{
public:
  struct Hit final
  {
    double distance = std::numeric_limits<double>::infinity();

    std::uint32_t index = 0xffffffff;

    operator bool() const noexcept { return distance != std::numeric_limits<double>::infinity(); }
  };

  BvhModel(std::uint32_t point_count);

  BvhModel(const BvhModel&) = delete;

  BvhModel(BvhModel&&) = delete;

  BvhModel& operator=(const BvhModel&) = delete;

  BvhModel& operator=(BvhModel&&) = delete;

  ~BvhModel();

  void commit();

  void set_point(std::uint32_t index, double x, double y, double z);

  void set_radius(std::uint32_t index, double r);

  Hit intersect(double ox, double oy, double oz, double dx, double dy, double dz) const;

private:
  BvhModelImpl& self() { return *self_; }

private:
  BvhModelImpl* self_ = nullptr;
};
