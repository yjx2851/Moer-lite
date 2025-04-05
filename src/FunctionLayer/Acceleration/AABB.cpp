#include "AABB.h"

Point3f minP(const Point3f &p1, const Point3f &p2) {
  return Point3f{std::min(p1[0], p2[0]), std::min(p1[1], p2[1]),
                 std::min(p1[2], p2[2])};
}

Point3f maxP(const Point3f &p1, const Point3f &p2) {
  return Point3f{std::max(p1[0], p2[0]), std::max(p1[1], p2[1]),
                 std::max(p1[2], p2[2])};
}

AABB AABB::Union(const AABB &other) const {
  Point3f min = minP(other.pMin, pMin), max = maxP(other.pMax, pMax);
  return AABB{min, max};
}

void AABB::Expand(const AABB &other) {
  pMin = minP(pMin, other.pMin);
  pMax = maxP(pMax, other.pMax);
}

AABB AABB::Union(const Point3f &other) const {
  Point3f min = minP(other, pMin), max = maxP(other, pMax);
  return AABB{min, max};
}

void AABB::Expand(const Point3f &other) {
  pMin = minP(pMin, other);
  pMax = maxP(pMax, other);
}

bool AABB::Overlap(const AABB &other) const {
  for (int dim = 0; dim < 3; ++dim) {
    if (pMin[dim] > other.pMax[dim] || pMax[dim] < other.pMin[dim]) {
      return false;
    }
  }
  return true;
}



bool AABB::RayIntersect(const Ray &ray, float *tMin, float *tMax) const {
  //* todo 实现AABB与光线求交
  float t0 = ray.tNear;
  float t1 = ray.tFar;

  // 遍历 x, y, z 三个轴
  for (int i = 0; i < 3; ++i) {
      float invD = 1.0f / ray.direction[i];
      float tNear = (pMin[i] - ray.origin[i]) * invD;
      float tFar  = (pMax[i] - ray.origin[i]) * invD;

      // 如果方向是负的，交换 near 和 far
      if (invD < 0.0f) std::swap(tNear, tFar);

      // 更新整体的交区间
      t0 = std::max(t0, tNear);
      t1 = std::min(t1, tFar);

      // 区间无交，返回false
      if (t0 > t1) return false;
  }

  if (tMin) *tMin = t0;
  if (tMax) *tMax = t1;
  return true;
}

Point3f AABB::Center() const {
  return Point3f{(pMin[0] + pMax[0]) * .5f, (pMin[1] + pMax[1]) * .5f,
                 (pMin[2] + pMax[2]) * .5f};
}