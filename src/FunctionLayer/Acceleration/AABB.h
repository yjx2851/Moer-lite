#pragma once
#include <CoreLayer/Math/Math.h>
#include <FunctionLayer/Ray/Ray.h>
// Axis-aligned Bounding box，即AABB
class AABB {
public:
  AABB() {
    pMin = Point3f{FLT_MAX, FLT_MAX, FLT_MAX};
    pMax = Point3f{-FLT_MAX, -FLT_MAX, -FLT_MAX};
  }

  AABB(const Point3f &_pMin, const Point3f &_pMax) : pMin(_pMin), pMax(_pMax) {}

  int MaximumExtent() const {
    Vector3f diag = pMax - pMin;
    if (diag[0] > diag[1] && diag[0] > diag[2])
        return 0; // x轴最大
    else if (diag[1] > diag[2])
        return 1; // y轴最大
    else
        return 2; // z轴最大
}

  AABB Union(const AABB &other) const;

  void Expand(const AABB &other);

  AABB Union(const Point3f &other) const;

  void Expand(const Point3f &other);

  bool Overlap(const AABB &other) const;

  bool RayIntersect(const Ray &ray, float *tMin = nullptr,
                    float *tMax = nullptr) const;

  Point3f Center() const;

public:
  Point3f pMin, pMax;
};