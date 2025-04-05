#pragma once
#include "Acceleration.h"
class BVH : public Acceleration{
public:
    BVH() = default;
    void build() override;
    bool rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const override;
    struct BVHNode;
    struct BVHShapeInfo;
    BVHNode * root;
    BVHNode * recursiveBuild(std::vector<BVHShapeInfo> &shapeInfos, int l, int r, std::vector<std::shared_ptr<Shape>> &orderedShapes);
protected:
    static constexpr int bvhLeafMaxSize = 64;
};
